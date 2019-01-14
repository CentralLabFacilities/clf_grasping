#!/usr/bin/env python

import sys
import rospy

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import PointCloud2

from clf_object_recognition_msgs.srv  import Detect3D
from clf_grasping_msgs.srv import CloudToCollision

def classify_3d():
    srv = "/detect_objects"
    print("waiting for service " + srv)
    rospy.wait_for_service(srv)
    print("detecting objects...")
    try:
        client = rospy.ServiceProxy(srv, Detect3D)
        resp = client()
        print("detection done")
        return resp.detections
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def add_to_planning_scene(objects):
    scene_diff = PlanningScene()
    scene_diff.world.collision_objects = objects
    scene_diff.is_diff = True
    srv = "/apply_planning_scene"
    print("waiting for service " + srv)
    rospy.wait_for_service(srv)
    print("adding " + str(len(objects)) + " objects ...")
    try:
        apply = rospy.ServiceProxy(srv, ApplyPlanningScene)
        print(scene_diff)
        resp = apply(scene_diff)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def send_clouds(detections):
    print("publishing clouds")
    pub = rospy.Publisher('/clouds', PointCloud2, queue_size=10)
    for detect3d in detections:
        print(detect3d.source_cloud)
        pub.publish(detect3d.source_cloud)

def fit_objects(detections, shapes = []):
    co_objects = []
    i = 1
    srv = "/cloud_to_co"
    print("waiting for service " + srv)
    rospy.wait_for_service(srv)
    print("fitting objects ...")
    client = rospy.ServiceProxy(srv, CloudToCollision)
    for detect3d in detections:
        objectid = ""
        if len(detect3d.results) > 0:
            for hyp in detect3d.results:
                objectid = objectid + str(hyp.id) + ";"
        else:
            objectid = "unknown" + str(i)
            i += 1
        print(" fitting object: " + objectid)
        try:
            resp = client(shapes,detect3d.source_cloud)
            co = resp.collision_object
            
            # Fill co
            co.id = objectid
            #co.operation = CollisionObject.ADD
            co.header = detect3d.source_cloud.header

            co_objects.append(co)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    print("fitting finished")
    return co_objects


if __name__ == "__main__":
    rospy.init_node('clouds', anonymous=True)
    classes = classify_3d()
    print("got " + str(len(classes)) + " objects classified")
    send_clouds(classes)
    co_objects = fit_objects(classes)
    add_to_planning_scene(co_objects)
