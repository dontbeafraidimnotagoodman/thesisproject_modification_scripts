#!/usr/bin/env python

import numpy as np
import rospy
import tf


rospy.init_node('cftarget')
tf = tf.TransformListener()
tf.waitForTransform("/world", "/vicon/cftarget_1/cftarget_1", rospy.Time(0), rospy.Duration(0.1))

position, quaternion = tf.lookupTransform("/world", "/vicon/cftarget_1/cftarget_1", rospy.Time(0))

print(position)
