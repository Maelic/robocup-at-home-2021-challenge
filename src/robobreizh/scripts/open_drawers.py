#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Grasping.grasping_node import Grasping
from utils import *
import rospy
import sys
import numpy as np
import tf
import json
from std_msgs.msg import String
from std_msgs.msg import Float32, Int64
from sensor_msgs.msg import PointCloud2, Image, PointCloud, PointField, JointState

from robobreizh.msg import CloudIndexed, CloudSources, GraspConfigList, DetectedObj, BoundingBoxCoord, GraspConfig, GraspServerRequest
from robobreizh.srv import detect_grasps, object_detection, grasping
import time
import moveit_commander
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point32, Point, PoseStamped, Pose, Twist
from xml_utils import ObjectBrowserYolo
from tf import TransformListener
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
from objects import *

rospy.init_node("Drawer")

grasp_node = Grasping()
listener = tf.TransformListener()
yolo_object_browser = ObjectBrowserYolo('/workspace/src/robobreizh/scripts/obj_yolo.xml')

def moveto_right_drawers():
	move_base_goal(0.0,0.40,-90)

def moveto_left_drawers():
	move_base_goal(0.45,0.30,-90)

def move_arm(joints_pose):
	group_name = "arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	move_group.allow_replanning(True)
	#move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
	joint_goal = move_group.get_current_joint_values()

	joints_index = {'arm_lift_joint':0, 'arm_flex_joint':1, 'arm_roll_joint':2, 'wrist_flex_joint':3, 'wrist_roll_joint':4, 'wrist_ft_sensor_frame_joint':5, 'wrist_ft_sensor_frame_inverse_joint':6, 'hand_palm_joint':7,}
	joints_index[0] = 0.6
	 
	for i in range(len(joints_pose)):
		if joint == 0:
			joints_pose[i] = joint_goal[i]

	move_group.go(joints_pose, wait=True)
	move_group.stop()

def move_arm_drawer_bottom():
	joints_pose = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	joints_pose[3] = 0.2

	move_arm(joints_pose)

def move_arm_drawer_top():
	joints_pose = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

	joints_pose[3] = 0.2

	move_arm(joints_pose)

def open_drawers():
	
	# 1. Open right bottom drawer

	moveto_right_drawers()
	move_arm_drawer_bottom()
	move_hand(0)
	move_base_vel(0.3,0.0,0.0)
	move_hand(1)
	move_base_vel(-0.3,0.0,0.0)

	# 2. Open right top drawer

	move_arm_drawer_top()
	move_hand(0)
	move_base_vel(0.3,0.0,0.0)
	move_hand(1)
	move_base_vel(-0.3,0.0,0.0)

	# 3. Open bottom left drawer
	
	moveto_left_drawers()
	move_arm_drawer_bottom()
	move_hand(0)
	move_base_vel(0.3,0.0,0.0)
	move_hand(1)
	move_base_vel(-0.3,0.0,0.0)


# Get grasping pose
def get_obj_pose():
	move_head_tilt(-0.8)
	moveto_right_drawers()

	rospy.loginfo("Waiting for Grasping service...")
	start = time.time()
	rospy.wait_for_service('/detect_grasps_server/detect_grasps')
	grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

	# Drawer right top
	taille_zone_drawer_right_top = [250, 100, 350, 180]
	boundingbox = BoundingBoxCoord()
	boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in taille_zone_drawer_right_top)
	# Dimension de la zone dans laquelle tu veux detecter quelque chose, 
	# coordonnes xmin, ymin, xmax, ymax de l'image (point en haut a gauche et en bas a droite qui forme un rectangle)
	# par defaut j'ai mis toute l'image, attention plus la zone est grande plus c'est lourd a executer

	msg = GraspServerRequest()
	msg.bounding_box = boundingbox
	
	msg.global_cloud = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
	try:
		resp2 = grasp_service(msg)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

	print("Time to find a good grasp: {}".format(time.time() - start))

	detected_grasp = GraspConfigList()
	detected_grasp = resp2.grasp_configs.grasps
	best_grasp = GraspConfig()
	best_grasp = detected_grasp[0]

	# Execution du grasp
	grasp_node = Grasping()

	if grasp_node.grasp_ground(best_grasp):
		move_hand(0)
		print("Grasp successful!")
	else:
		print("Grasp failed!")

	# Ici tu fais ce que tu veux de l'objet dans ta main...

	move_head_tilt(0.0)

	# Move back arm for easy navigation
	move_arm_init()

if __name__ == "__main__":
	move_arm_init()

	get_obj_pose()