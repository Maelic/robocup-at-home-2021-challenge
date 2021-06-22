#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math

from utils import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
#from Vision import detection_node

if __name__=='__main__':
    rospy.init_node('test_vision')

    put_object("cracker_box", 0.6, 1.9, 0.5)
    put_object("mustard_bottle", 1.0, 1.8, 0.5)
    put_object("mug", 0.8, 1.7, 0.5)
    put_object("foam_brick", 1.2, 1.7, 0.5)

    try:
        # look down a little
        move_head_tilt(-0.4)
    except:
        rospy.logerr('fail to init')
        sys.exit()

    try:
        # move position
        move_arm_init()
        # move in front of the long table
        move_base_goal(1, 1.0, 90)
    except:
        rospy.logerr('fail to move')
        sys.exit()
