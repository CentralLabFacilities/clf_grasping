#!/usr/bin/env python

import rospy

from sq_fitting_adapter.vision import ObjectFitter

rospy.init_node("object_fitter")
object_fitter = ObjectFitter()
rospy.spin()
