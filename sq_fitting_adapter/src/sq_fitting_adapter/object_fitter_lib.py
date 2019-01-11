#!/usr/bin/env python

import argparse
import rospy
import numpy as np
from copy import deepcopy
import logging

from sq_fitting_ros.srv import SqFitPointCloud, SqFitPointCloudRequest, SqFitPointCloudResponse
from sq_fitting_ros.srv import SqFitPointCloud2, SqFitPointCloud2Request, SqFitPointCloud2Response

from tf.transformations import quaternion_matrix

from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal,\
    FindGraspableObjectsFeedback, FindGraspableObjectsResult,\
    FitPrimitivesAction, FitPrimitivesFeedback, FitPrimitivesResult,\
    GraspableObject, Object as FitObject

from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

import actionlib

import tf2_ros

# RSB
RSB_SUPPORT = True
try:
    import rsb
    import rsb.converter
    from rst.geometry.PointCloudSet3DFloat_pb2 import PointCloudSet3DFloat

    from rst.tracking.TrackedClassifiedRegions3D_pb2 import TrackedClassifiedRegions3D
    from rst.tracking.TrackedClassifiedRegion3D_pb2 import TrackedClassifiedRegion3D
    from rst.tracking.TrackingInfo_pb2 import TrackingInfo
    from rst.geometry.PolygonalPatch3DSet_pb2 import PolygonalPatch3DSet
except ImportError:
    print "disabling rsb support"
    RSB_SUPPORT = False

supported_shape_preference = "cbs"


def process_clouds(event, ros_pcls, cloud_frame):
    '''
    process rsb received point cloud in a ros pointcloud list
    '''

    pcs = event.getData()
    num = len(pcs.clouds)
    if num == 0:
        return False
    del ros_pcls[:]
    print("Received %s clouds" % num)

    for cloud in pcs.clouds:
        ros_pcl = PointCloud()
        ros_pcl.header.frame_id = cloud_frame
        ros_pcl.header.stamp = rospy.Time.now()

        for p in cloud.points:
            ros_pcl.points.append(Point32(p.x, p.y, p.z))
        ros_pcls.append(deepcopy(ros_pcl))
    return True


def process_clouds2(event, ros_pcls, cloud_frame):
    '''
    process rsb received point cloud in a ros pointcloud2 list
    '''
    pcs = event.getData()
    num = len(pcs.clouds)
    if num == 0:
        return False
    del ros_pcls[:]
    print("Received %s clouds" % num)

    hd = Header()
    hd.frame_id = cloud_frame
    hd.stamp = rospy.Time.now()

    for cloud in pcs.clouds:
        # need to create an intermediate list because other method does not work
        points = []
        for p in cloud.points:
            points.append([p.x, p.y, p.z])
        ros_pcl2 = pc2.create_cloud_xyz32(hd, points)
        # this method does not work due to ros wanting to do a len on the generator
        # ros_pcl2 = pc2.create_cloud_xyz32(hd, ([p.x, p.y, p.z] for p in cloud.points))
        ros_pcls.append(deepcopy(ros_pcl2))
    return True


def process_regions3d(event, object_uids, object_centers):
    '''
    process rsb received object classification3d in a local variable
    '''

    classified_regions3d = event.getData()
    num = len(classified_regions3d.region)  # rst type is not coherent and should be regionS
    del object_uids[:]
    object_centers.clear()
    print("Received %s regions3d" % num)

    for region in classified_regions3d.region:
        # store id
        object_uids.append(region.info.id)
        # store center of the bounding box
        object_centers[region.info.id] = np.array([region.region.region.transformation.translation.x,
                                                   region.region.region.transformation.translation.y,
                                                   region.region.region.transformation.translation.z])
    print object_centers


def process_table(event, table_planes, table_safety_margin=0.0):
    '''
    process rsb received tables classification in a local variable
    '''

    tables = event.getData().patches
    table_planes.clear()
    # get table normal
    for i, table in enumerate(tables):
        R = quaternion_matrix([table.base.rotation.qx,
                               table.base.rotation.qy,
                               table.base.rotation.qz,
                               table.base.rotation.qw])
        zvect = R[0:3, 2]
        # print "zvect", zvect

        # get dist to table center
        vect_to_table_center = np.asarray([table.base.translation.x,
                                           table.base.translation.y,
                                           table.base.translation.z])
        # print "vect_to_table_center ", vect_to_table_center
        # compute distance to the plane from the reference (=from 0,0,0)
        d = np.fabs(np.dot(vect_to_table_center, zvect))
        print "distance to table plane: ", d
        if table_safety_margin is not None:
            d += table_safety_margin
        # used for fitting (distance to plane is actually an offset and should be minus d by convention)
        table_planes["table" + str(i)] = np.concatenate([zvect, np.asarray([-d])])


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

    if RSB_SUPPORT:
        # RSB converters
        converter_pcl = rsb.converter.ProtocolBufferConverter(messageClass=PointCloudSet3DFloat)
        rsb.converter.registerGlobalConverter(converter_pcl)
        converter_clreg3d = rsb.converter.ProtocolBufferConverter(messageClass=TrackedClassifiedRegions3D)
        rsb.converter.registerGlobalConverter(converter_clreg3d)
        converter_table = rsb.converter.ProtocolBufferConverter(messageClass=PolygonalPatch3DSet)
        rsb.converter.registerGlobalConverter(converter_table)

        print("Registered converter %s" % rsb.converter)
        print("Registered converters:\n%s " % rsb.converter.getGlobalConverterMap(bytearray))

    logging.basicConfig()
    maxSphereRadius = 5.0

    def __init__(self, name, use_rsb=True):

        # create messages that are used to publish feedback/result
        if RSB_SUPPORT and use_rsb:
            self.feedback = FindGraspableObjectsFeedback()
            self.result = FindGraspableObjectsResult()
        else:
            self.feedback = FitPrimitivesFeedback()
            self.result = FitPrimitivesResult()

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
        self.init_com(use_rsb)

        # TF listener
        self.tf_Buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_Buffer)

        # ROS actionlibs
        if RSB_SUPPORT and use_rsb:
            self._action_name = name
            self._as = actionlib.SimpleActionServer(self._action_name, FindGraspableObjectsAction,
                                                    execute_cb=self.execute_find_cb, auto_start=False)
        else:
            self._action_name = name
            self._as = actionlib.SimpleActionServer(self._action_name, FitPrimitivesAction,
                                                    execute_cb=self.execute_fit_cb, auto_start=False)

        self._as.start()
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
            self._camera_frame = "unknown_camera_link"

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

    def init_com(self, use_rsb):
        # init communication, services and actionlibs
        if RSB_SUPPORT and use_rsb:

            print "wait_for_service '/fit_sq'"
            rospy.wait_for_service('/fit_sq')
            # ROS client
            self.sq_client = rospy.ServiceProxy('/fit_sq', SqFitPointCloud)
            print "found service '/fit_sq'"

            # RSB handler
            self.listener = rsb.createListener("/clafu/results/cloud")
            self.listener.addHandler(self.handle_clouds)
            print "created listener to cloud"

            self.listener_region3d = rsb.createListener("/clafu/results/region3d")
            self.listener_region3d.addHandler(self.handle_regions3d)
            print "created listener to region2d"

            self.listener_table = rsb.createListener("/clafu/table")
            self.listener_table.addHandler(self.handle_table)
            print "created listener to table"
        else:
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
        if self.tf_Buffer.can_transform(frame_orig, frame_dest, rospy.Time(), rospy.Duration(1.0)):
            try:
                tf_Stamped = self.tf_Buffer.lookup_transform(frame_orig, frame_dest, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                rospy.logdebug("getTFPos lookup frame orig " + str(frame_orig) + " to " + str(frame_dest) + " transform not found ")
                rospy.logdebug(e)
                return None
            except tf2_ros.ExtrapolationException, e:
                rospy.logdebug("getTFPos extrapolation frame orig " + str(frame_orig) + " to " + str(frame_dest) + " transform not found ")
                rospy.logdebug(e)
                return None
            return np.array([tf_Stamped.transform.translation.x * 100.0,
                             tf_Stamped.transform.translation.y * 100.0,
                             tf_Stamped.transform.translation.z * 100.0])
        else:
            rospy.logdebug("getTFPos frame dest " + str(frame_dest) + " doest not exist")
            return None

    def create_object_from_sq(self, sq_params):
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

        obj = FitObject()
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    def process_pointcloud(self, pcl, plane=None):
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
            req.shape_preference = self.shape_preference
            source_frame_id = pcl.header.frame_id
            if source_frame_id == "":
                rospy.logwarn("pcl frame_id empty, forcing to base_link")
                source_frame_id = self.cloud_frame
            camera_pos = self.getTFPos(source_frame_id, self.camera_frame)
            if camera_pos is not None:
                req.camera_pose.pose.position = Point(camera_pos[0], camera_pos[1], camera_pos[2])
                req.camera_pose.header.frame_id = source_frame_id
            else:
                rospy.logwarn("camera pose not found, forcing to [0.4, 0, 1.35] in /base_link")
                req.camera_pose.pose.position = Point(40, 0, 135)
                req.camera_pose.header.frame_id = "base_link"

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

    def handle_clouds(self, event):
        self.print_debug("Received cloud event")
        self.clafu_updated = process_clouds(event, self.ros_pcls, self.cloud_frame)

    def handle_regions3d(self, event):
        self.print_debug("Received region3d event")
        process_regions3d(event, self.object_uids, self.object_centers)

    def handle_table(self, event):
        self.print_debug("Received table event")
        process_table(event, self.table_planes, self.table_safety_margin)

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

    def get_center_from_properties(self, obj_props):
        '''
        extract center of object stored in properties
        '''
        center = np.array([0.0, 0.0, 0.0])
        found_position = False
        for key, value in obj_props:
            for i, attrib in enumerate(['x', 'y', 'z']):
                if "position." + attrib in key:
                    center[i] = value
                    found_position = True
        if not found_position:
            self.print_debug("no center found in properties")
            raise ValueError("No property position")
        return center

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

    def execute_find_cb(self, goal):
        """ main actionlib callback processing separately received pointcloud data (over RSB)"""
        success = True

        if (len(self.ros_pcls) > 0 and len(self.ros_pcls) == len(self.object_uids)):

            self.feedback.object = GraspableObject()

            if self.clafu_updated:
                self.result.objects = []
                self.result.support_surfaces = []

                for uid, cloud in zip(self.object_uids, self.ros_pcls):
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        success = False
                    else:
                        sq_params = []
                        # find the closest supporting plane for this object
                        (plane, plane_name) = self.find_closest_plane(self.object_centers[uid])
                        # fit the pcl
                        sq_params = self.process_pointcloud(cloud, plane)

                        if len(sq_params.data) > 0:
                            # create an object from the fitting
                            grasp_obj = GraspableObject()
                            grasp_obj.object = self.create_object_from_sq(sq_params.data)
                            grasp_obj.object.name = str(uid)
                            grasp_obj.object.header.frame_id = cloud.header.frame_id
                            # if the closest plane was found
                            if plane is not None:
                                grasp_obj.object.surface.coef = plane
                        else:
                            success = False
                            break

                        # publish feedback
                        self.feedback.object = grasp_obj
                        self._as.publish_feedback(self.feedback)
                        # store intermediate result
                        self.result.objects.append(grasp_obj)

                # pass over all the planes/table found
                for table_name in self.table_planes:
                    additional_obj = FitObject()
                    additional_obj.surface.coef = self.table_planes[table_name]
                    self.result.support_surfaces.append(additional_obj)

                # indicated we processed the data
                self.clafu_updated = False

            # else use previous result and resend it

            # publish result
            if success:
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self.result)
        else:
            rospy.logwarn("cannot process incomplete data received: %d clouds, %d regions",
                          len(self.ros_pcls), len(self.object_uids))
            self._as.set_aborted()

    def execute_fit_cb(self, goal):
        """ main actionlib callback processing directly received pointcloud data"""
        success = True

        if (len(goal.objects) > 0):
            # check config_names and adapt if none, one or all
            config_names = goal.config_names
            if len(config_names) == 0 or (len(goal.objects) != len(config_names) and len(config_names) > 1):
                config_names = ['default']*len(goal.objects)
            if len(goal.config_names) == 1:
                if len(goal.config_names[0]) == 0:  # empty string
                    config_names = ['default']*len(goal.objects)
                else:
                    config_names = [goal.config_names[0]]*len(goal.objects)
            self.print_debug("using configs :" + str(config_names))
            # load support planes
            if len(goal.support_surfaces) > 0:
                self.store_tables(goal.support_surfaces)
            self.feedback.object = FitObject()
            self.result.objects = []

            # for each object / config_name pair
            for input_obj, config_name in zip(goal.objects, config_names):
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                else:
                    sq_params = []
                    # set the config
                    if not self.setConfig(config_name):
                        success = False
                        rospy.logwarn("object_fitter: Failed to configure")
                        self._as.set_aborted()
                        break
                    else:

                        # get table information if available
                        if input_obj.support_surface != "" and input_obj.support_surface in self.table_planes:
                            plane = deepcopy(self.table_planes[input_obj.support_surface])
                            support_name = input_obj.support_surface
                            self.print_debug("using given support surface: " + input_obj.support_surface)
                        else:  # find closest table
                            try:  # extract object center if exists
                                obj_center = self.get_center_from_properties(input_obj.properties)
                            except:  # else recompute center from point_cloud
                                obj_center = self.compute_center_from_pcl(input_obj.point_cluster)
                            # find the closest supporting plane for this object
                            (plane_org, support_name) = self.find_closest_plane(obj_center)
                            self.print_debug("using closest support surface: " + support_name)
                            plane = deepcopy(plane_org)

                        # distance to plane is actually an offset and should be minus d by convention)
                        # so substract table margin (because distance is probably negative)
                        
                        plane[3] -= self.table_safety_margin
                        # fit the pcl
                        sq_params = self.process_pointcloud(input_obj.point_cluster, plane)

                        if sq_params is not None and len(sq_params.data) > 0:
                            # create an object from the fitting
                            obj = self.create_object_from_sq(sq_params.data)
                            obj.name = input_obj.name
                            obj.support_surface = support_name
                            # TODO: if object_frame_id is not the same as the point_cluster frame id
                            # one transform the pose of the primitive to be correct
                            # for now copy the object.frame_id
                            obj.header.frame_id = input_obj.header.frame_id
                            if self.copy_original_data:
                                obj.point_cluster = deepcopy(input_obj.point_cluster)
                        else:
                            # TODO think if one should create a default box around the cluster at least
                            # don't fail, just don't add this object
                            break

                    # publish feedback
                    self.feedback.object = obj
                    self._as.publish_feedback(self.feedback)
                    # store intermediate result
                    self.result.objects.append(obj)

            # publish result
            if success:
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self.result)
        else:
            rospy.logwarn("cannot process data received: no object given")
            self._as.set_aborted()
