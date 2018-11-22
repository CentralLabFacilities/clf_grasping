#!/usr/bin/env python

import sys
import rospy
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

def __make_cylinder(name, pose, size):
	co = CollisionObject()
	co.operation = CollisionObject.ADD
	co.id = name
	co.header = pose.header
	cylinder = SolidPrimitive()
	cylinder.type = SolidPrimitive.CYLINDER
	cylinder.dimensions = list(size)
	co.primitives = [cylinder]
	co.primitive_poses = [pose.pose]
	return co

def __make_box(name,pose,size):
	co = CollisionObject()
	co.operation = CollisionObject.ADD
	co.id = name
	co.header = pose.header
	box = SolidPrimitive()
	box.type = SolidPrimitive.BOX
	box.dimensions = list(size)
	co.primitives = [box]
	co.primitive_poses = [pose.pose]
	return co

def spawnObjects(objects):
	scene_diff = PlanningScene()
	scene_diff.world.collision_objects = objects
	scene_diff.is_diff = True

	rospy.wait_for_service('/apply_planning_scene')
	try:
		apply = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
		resp = apply(scene_diff)
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def createTable(px,py,pz,w,l):
	pose = PoseStamped()
	pose.header.frame_id = 'base_link'
	pose.pose.position.x = px
	pose.pose.position.y = py
	pose.pose.position.z = pz
	return __make_box('table',pose,(w,l,0.03))

def createObject(px,py,pz,h,r):
	pose = PoseStamped()
	pose.header.frame_id = 'base_link'
	pose.pose.position.x = px
	pose.pose.position.y = py
	pose.pose.position.z = pz
	return __make_cylinder('object',pose,(h,r))

def addObjects():
	objects = []
	objects.append(createObject(0.64,0,0.52,0.18,0.02))
	objects.append(createTable(0.78,0,0.42,0.4,1))
	spawnObjects(objects)

if __name__ == "__main__":
	rospy.init_node('spawn_table')
	addObjects()