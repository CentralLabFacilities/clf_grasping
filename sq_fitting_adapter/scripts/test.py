#!/usr/bin/env python

import sys
import rospy

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

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
        return resp.detections.detections
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def add_to_planning_scene(objects):
    scene_diff = PlanningScene()
    scene_diff.world.collision_objects = objects
    scene_diff.is_diff = True
    srv = "/apply_planning_scene"
    print("waiting for service " + srv)
    rospy.wait_for_service(srv)
    print("adding objects ...")
    try:
        apply = rospy.ServiceProxy(srv, ApplyPlanningScene)
        resp = apply(scene_diff)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

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
            objectid = "unknown" + i
            i += 1
        print("fitting object: " + objectid)
        try:
            resp = client(shapes,detect3d.source_cloud)
            co = resp.collision_object
            
            # Fill co
            co.id = objectid
            #co.operation = CollisionObject.ADD

            co_objects.append(co)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    return co_objects


if __name__ == "__main__":
    classes = classify_3d()
    print("got " + str(len(classes)) + " objects classified")
    co_objects = fit_objects(classes)
    add_to_planning_scene(co_objects)