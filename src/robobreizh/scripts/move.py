#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
from utils import *
#from Vision.vision import Vision
import time

def demo():
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
        move_base_goal(1, 0.5, 90)
    except:
        rospy.logerr('fail to move')
        sys.exit()

    try:
        # neutral position
        move_arm_neutral()
        # open hand
        move_hand(1)
        # move hand toward (we need to detect object here)
        move_wholebody_ik(0.9, 1.5, 0.2, 180, 0, 90)
        # lower down the hand
        move_wholebody_ik(0.9, 1.5, 0.08, 180, 0, 90)
        # close hand
        move_hand(0)
        # neutral position
        move_arm_neutral()
    except:
        rospy.logerr('fail to grasp')
        sys.exit()

    try:
        # move position
        move_arm_init()
        # move in front of the tray
        move_base_goal(1.8, -0.1, -90)
        # neutral position
        move_arm_neutral()
        # open hand
        move_hand(1)
    except:
        rospy.logerr('fail to move')
        sys.exit()

def move_head():
    try:
        # look down a little
        move_head_tilt(-0.4)
    except:
        rospy.logerr('fail to init')
        sys.exit()

def demo_grasping():
    put_object("cracker_box", 0.6, 1.9, 0.5)
    put_object("mustard_bottle", 1.0, 1.8, 0.5)
    put_object("mug", 0.8, 1.7, 0.5)
    put_object("foam_brick", 1.2, 1.7, 0.5)

	
	# Move toward the table
	# look down a little
    move_head_tilt(-0.5)

	# move position
    move_arm_init()
	# move in front of the long table
    move_base_goal(1, 1, 90)
	# Look for objects
    time.sleep(1)
    #vision_module = Vision()
    #objects = vision_module.searchObject()
    
if __name__=='__main__':
    rospy.init_node('move')

    demo_grasping()
 
