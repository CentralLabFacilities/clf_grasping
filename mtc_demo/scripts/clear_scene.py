#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty

def clearScene():
	rospy.wait_for_service('/mtc_demo/clear_planning_scene')
	try:
		clear = rospy.ServiceProxy('/mtc_demo/clear_planning_scene', Empty)
		resp = clear()
		return resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
    clearScene()