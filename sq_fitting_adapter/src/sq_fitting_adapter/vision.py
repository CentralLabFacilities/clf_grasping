#!/usr/bin/env python

import argparse
import rospy
import numpy as np
from copy import deepcopy
import logging

from sq_fitting_ros.srv import SqFitPointCloud, SqFitPointCloudRequest, SqFitPointCloudResponse
from sq_fitting_ros.srv import SqFitPointCloud2, SqFitPointCloud2Request, SqFitPointCloud2Response

from tf.transformations import quaternion_matrix

from moveit_msgs.msg import CollisionObject

from clf_grasping_msgs.srv import CloudToCollision, CloudToCollisionRequest, CloudToCollisionResponse

from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

import actionlib

import tf2_ros

supported_shape_preference = "cbs"

class ObjectFitter(object):
    '''
    Object fitter class to centralize object fitting
    '''

    # mandatory params
    _parameters_names = ["camera_frame", "shape_preferences"]
    # others  "default_table_height", "copy_original_data"
    _shape_preferences_map = {"box": 'b',
                              "flatcylinder": 'f',
                              "cylinder": 'c',
                              "ellipsoid": 'e',
                              "sphere": 's',
                              "any": 'a'}

    # defaults
    _copy_original_data = True
    _default_table_height = 0.6

    logging.basicConfig()
    maxSphereRadius = 5.0

    def __init__(self):

        # init internals
        self.ros_pcls = []
        self.object_uids = []
        self.object_centers = {}
        self.table_height = self._default_table_height
        self.shape_preference = "cbs"
        self.copy_original_data = self._copy_original_data
        self.table_planes = {}
        self.table_planes['default'] = np.array([0.0, 0.0, 1.0, -self.table_height])
        self.clafu_updated = False

        # init configs
        self.init_configs()

        # init communication
        self.init_com()

        # TF listener
        self.tf_Buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_Buffer)

        self._server = rospy.Service('cloud_to_co', CloudToCollision, self.cb_cloud_to_co)
        print "ready to fit pointclouds"

    def init_configs(self):
        # load some default params
        table_safety_margin = rospy.get_param('~table_safety_margin', 0.0)
        if table_safety_margin != 0.0:
            self._table_safety_margin = table_safety_margin
        else:
            self._table_safety_margin = None

        camera_frame = rospy.get_param('~camera_frame', "")
        if camera_frame != "":
            self._camera_frame = camera_frame
        else:
            self._camera_frame = "xtion_depth_optical_frame"

        self._cloud_frame = rospy.get_param('~cloud_frame', "base_link")

        self._default_verbose = 1
        self.verbose = self._default_verbose

        # set some variable to their defaults
        self.cloud_frame = self._cloud_frame
        self.camera_frame = self._camera_frame
        self.table_safety_margin = self._table_safety_margin

        self.configs = {}
        # load configurations from parameter server
        if rospy.has_param('object_fitter_config'):
            self.configs = rospy.get_param('object_fitter_config')
        else:
            rospy.logerr("No configuration found, using default values")
            # self.configs

        # validate configs
        if self.checkConfigs():
            self.print_debug("All configurations are valid")
        else:
            rospy.logwarn("Invalid configuration were given, and pruned from the list")

    def init_com(self):
        
        print "wait_for_service '/fit_sq_pc2'"
        rospy.wait_for_service('/fit_sq_pc2')
        # ROS client
        self.sq_client = rospy.ServiceProxy('/fit_sq_pc2', SqFitPointCloud2)
        print "found service '/fit_sq_pc2'"

    def print_debug(self, string):
        if self.verbose:
            print string

    def setConfig(self, config_name):
        if config_name in self.configs:
            if len(self.configs[config_name]):
                # if exist extract verbose level
                if "verbose_level" in self.configs[config_name]:
                    self.verbose = self.configs[config_name]["verbose_level"]
                else:
                    self.verbose = self._default_verbose

                self.shape_preference = self.getCompactShapePref(self.configs[config_name]['shape_preferences'])
                self.camera_frame = self.configs[config_name]['camera_frame']
                self.cloud_frame = self.configs[config_name]['cloud_frame']
                self.table_safety_margin = self.configs[config_name]['table_safety_margin']
                self.copy_original_data = self.configs[config_name]['copy_original_data']
                self.table_height = self.configs[config_name]['default_table_height']
                self.table_planes['default'] = np.array([0.0, 0.0, 1.0, -self.table_height])

                self.print_debug("object_fitter: config " + config_name + " selected")
                return True
            else:
                rospy.logwarn("object_fitter: config " + config_name + " is empty")
                return False
        else:
            rospy.logwarn("object_fitter: config %s does not exist, use one of %s:", config_name,  map(str, self.configs.keys()))
            return False

    def checkConfigs(self):
        invalid_configs = []
        for config_name in self.configs:

            if not self.hasValidProp(config_name, invalid_configs):
                continue

            else:
                # check shape_pref and set all if needed
                sp = self.configs[config_name]['shape_preferences']
                if len(sp) == 0 or sp[0].lower() == "all":
                    sp = self._shape_preferences_map.keys()
                    # store it back
                    self.configs[config_name]['shape_preferences'] = sp

                # check camera frame
                if len(self.configs[config_name]['camera_frame']) == 0:  # use default
                    self.configs[config_name]['camera_frame'] = self._camera_frame

                # check cloud frame
                if "cloud_frame" not in self.configs[config_name]:  # use default
                    self.configs[config_name]['cloud_frame'] = self._cloud_frame

                # check table safety margin
                if "table_safety_margin" not in self.configs[config_name]:  # use default
                    self.configs[config_name]['table_safety_margin'] = self._table_safety_margin

                # check copy original data
                if "copy_original_data" not in self.configs[config_name] or type(self.configs[config_name]['copy_original_data']) is not bool:
                    self.configs[config_name]['copy_original_data'] = self._copy_original_data

                # check default table height
                if "default_table_height" not in self.configs[config_name]:
                    self.configs[config_name]['default_table_height'] = self._default_table_height

                rospy.loginfo("loaded " + config_name + " config")

        # cleanup invalid configs if any
        if len(invalid_configs) == len(self.configs):
            self.configs = {}
            return False
        if len(invalid_configs):
            for config_name in invalid_configs:
                del self.configs[config_name]
        return True

    def hasValidProp(self, config_name, invalid_configs):
        if not self.checkParams(config_name):
            invalid_configs.append(config_name)
            return False
        return True

    def checkParams(self, config_name):
        valid_param = True
        # check if all parameters are available
        for param_name in self._parameters_names:
            if param_name not in self.configs[config_name]:
                rospy.logwarn("missing parameter: " + param_name)
                valid_param = False
                break
        return valid_param

    def getCompactShapePref(self, shape_preferences):
        shape_preference_str = ""
        for shape_pref in shape_preferences:
            if shape_pref.lower() in self._shape_preferences_map:
                shape_preference_str += self._shape_preferences_map[shape_pref.lower()]
        self.print_debug("Using compact shape preference: " + shape_preference_str)
        return shape_preference_str

    def getTFPos(self, frame_orig, frame_dest):
        """
        use TF2 to get the current pos of a frame
        """
        rospy.logdebug("getTFPos source_frame %s target_frame %s, target_frame", frame_orig, frame_dest)
        if self.tf_Buffer.can_transform(frame_dest, frame_orig, rospy.Time(), rospy.Duration(1.0)):
            try:
                tf_Stamped = self.tf_Buffer.lookup_transform(frame_dest, frame_orig, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                rospy.logdebug("getTFPos lookup frame orig " + str(frame_orig) + " to " + str(frame_dest) + " transform not found ")
                rospy.logdebug(e)
                return None
            except tf2_ros.ExtrapolationException, e:
                rospy.logdebug("getTFPos extrapolation frame orig " + str(frame_orig) + " to " + str(frame_dest) + " transform not found ")
                rospy.logdebug(e)
                return None

            pose = Pose()
            pose.position.x = tf_Stamped.transform.translation.x
            pose.position.y = tf_Stamped.transform.translation.y
            pose.position.z = tf_Stamped.transform.translation.z
            pose.orientation = tf_Stamped.transform.rotation
            return pose
        else:
            rospy.logdebug("getTFPos frame dest " + str(frame_dest) + " doest not exist")
            return None

    def create_primitive_and_pose_from_sq(self, sq_params):
        """
        create a primitive object from superquadric parameters (no taper or bending)
        """
        primitive = SolidPrimitive()

        # size values are half the real dimensions
        size = sq_params[7:10]
        shape = sq_params[10:12]

        # pose
        pose = Pose()
        pose.position = Point(*sq_params[0:3])
        pose.orientation = Quaternion(*sq_params[3:7])

        # shape
        bEdgy = [i < 0.7 for i in shape]
        if not any(bEdgy) and np.std(size) / min(size) < 0.1:
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions.append(np.mean(size))

        elif not bEdgy[1] and abs(size[0] - size[1]) / min(size[0:2]) < 0.1:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions.append(2*size[2])
            primitive.dimensions.append(np.mean(size[0:2]))
        else:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions.append(2 * size[0])
            primitive.dimensions.append(2 * size[1])
            primitive.dimensions.append(2 * size[2])

        return (primitive,pose)

    def process_pointcloud(self, pcl, plane=None, shapes=None):
        '''
        process given point cloud, fit a superquadric, return sq_params
        '''
        is_pc2 = False
        if pcl:
            if type(pcl) is PointCloud2:
                is_pc2 = True
            if is_pc2:
                req = SqFitPointCloud2Request()
            else:
                req = SqFitPointCloudRequest()
            req.point_cloud = pcl
            if len(shapes) > 0:
                req.shape_preference = self.getCompactShapePref(shapes)
            else:
                req.shape_preference = self.shape_preference
            source_frame_id = pcl.header.frame_id
            if source_frame_id == "":
                rospy.logwarn("pcl frame_id empty, forcing to base_link")
                source_frame_id = self.cloud_frame
            camera_pos = self.getTFPos(source_frame_id, self.camera_frame)
            if camera_pos is not None:
                req.camera_pose.pose = camera_pos
                req.camera_pose.header.frame_id = source_frame_id
            else:
                rospy.logwarn("camera pose not found using [0,0,0] in pcl_frame_id " + source_frame_id)
                req.camera_pose.pose.position = Point(0, 0, 0)
                req.camera_pose.header.frame_id = source_frame_id
            print("camera:" + str(req.camera_pose))
            if plane is not None:
                req.table_plane.coef = plane
            try:
                # query fitting
                res = self.sq_client(req)
                return res.sq_params
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s" % e)

        if is_pc2:
            return SqFitPointCloud2Response().sq_params
        else:
            return SqFitPointCloudResponse().sq_params

    def find_closest_plane(self, center):
        """
        retrieves the closest plane to the given center
        """
        mindist = None
        closest_table = None
        for i, table_name in enumerate(self.table_planes):
            table = self.table_planes[table_name]
            # distance is projection of object translation on plane normal
            #  minus the table distance (which is stored negative)
            dist = np.dot(center, table[0:3]) + table[3]
            # only consider object above a plane
            if dist > 0:
                if mindist is not None:
                    if dist < mindist:
                        mindist = dist
                        closest_table = table_name
                else:
                    mindist = dist
                    closest_table = table_name

        if closest_table is not None:
            self.print_debug("table " + closest_table + " is the closes at distance " + str(mindist))
            return (self.table_planes[closest_table], closest_table)
        else:
            self.print_debug("No table found under the object")
            return (None, "")

    def store_tables(self, support_surfaces):
        '''
        store ros received tables in a local variable
        '''
        # fill up table_planes
        zeros = np.array([0.0, 0.0, 0.0, 0.0])
        for obj in support_surfaces:
            # if obj.surface.coef != [0, 0, 0, 0]:
            table = np.array(obj.surface.coef)
            if not np.array_equal(table, zeros):
                self.print_debug("adding table " + obj.name + " with coeff " + str(table))
                self.table_planes[obj.name] = table
            elif len(obj.primitives):  # a primitive represents the table, create a plane out of it
                # currently only handle the first primitive
                # only handle boxes
                if obj.primitives[0].type == 1:
                    dim = obj.primitives[0].dimensions  # [0.7705065906047821, 1.3, 0.01]
                    center = [obj.primitive_poses[0].position.x,
                              obj.primitive_poses[0].position.y,
                              obj.primitive_poses[0].position.z]
                    # TODO handle orientation of the object to create a correct normal to the plane
                    # should be 0.71 + 0.01/2
                    table = np.array([0, 0, 1.0, -1 * (center[2] + dim[2])])
                    self.print_debug("adding table from primitive " + obj.name + " with coeff " + str(table))
                    self.table_planes[obj.name] = table
            else:
                self.print_debug("no table found for surface " + obj.name)

    def compute_center_from_pcl(self, cluster):
        '''
        compute center of a point cloud
        '''
        size = cluster.height * cluster.width
        if size != 0:
            center = np.array([0.0, 0.0, 0.0])
            for p in pc2.read_points(cluster, field_names=("x", "y", "z"), skip_nans=True):
                center += np.array([p[0], p[1], p[2]])
            center = np.divide(center, float(size))
            return center
        else:
            return np.array([0.0, 0.0, 0.0])

    def cb_cloud_to_co(self, req):
        ret = CloudToCollisionResponse()

        plane = None
        sq_params = self.process_pointcloud(req.source_cloud, plane, req.shapes)
        if sq_params is not None and len(sq_params.data) > 0:
            (obj,pose) = self.create_primitive_and_pose_from_sq(sq_params.data)
            print("fitted object@:")
            print(pose.position)
        else:
            #TODO make cube
            print("could not fit object")
            raise Exception("could not fit any shapes")

        co = CollisionObject()
        co.primitives.append(obj)
        co.primitive_poses.append(pose)
        ret.collision_object = co

        return ret
