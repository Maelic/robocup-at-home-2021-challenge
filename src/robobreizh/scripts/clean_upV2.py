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
import signal
import math
import geometry_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy

#Deposits information
class Deposit():
	name = ""
	coord = [0,0,0]
	hand = Pose()


HAND_POSE_CONTAINER_A = Pose()
HAND_POSE_CONTAINER_A.position.x =  1.1096606162
HAND_POSE_CONTAINER_A.position.y = -0.558862732804
HAND_POSE_CONTAINER_A.position.z = 0.717533380345

HAND_POSE_CONTAINER_A.orientation.x = 0.76471035193
HAND_POSE_CONTAINER_A.orientation.y = 0.54119386278
HAND_POSE_CONTAINER_A.orientation.z = -0.270179522124
HAND_POSE_CONTAINER_A.orientation.w = 0.222104269127

HAND_POSE_CONTAINER_B = Pose()
HAND_POSE_CONTAINER_B.position.x = 1.37748081492
HAND_POSE_CONTAINER_B.position.y = -0.447429579289
HAND_POSE_CONTAINER_B.position.z = 0.595888279798

HAND_POSE_CONTAINER_B.orientation.x = 0.546604629508
HAND_POSE_CONTAINER_B.orientation.y = 0.471480417784
HAND_POSE_CONTAINER_B.orientation.z = -0.506663948606
HAND_POSE_CONTAINER_B.orientation.w = 0.471403476684

HAND_POSE_BIN_B = Pose()
HAND_POSE_BIN_B.position.x = 1.93789748557
HAND_POSE_BIN_B.position.y = -1.43153592659
HAND_POSE_BIN_B.position.z = 0.606634427137

HAND_POSE_BIN_B.orientation.x = 0.630930406779
HAND_POSE_BIN_B.orientation.y = -0.605703459114
HAND_POSE_BIN_B.orientation.z = 0.361888663505
HAND_POSE_BIN_B.orientation.w = 0.322624761365

HAND_POSE_BIN_A = Pose()
HAND_POSE_BIN_A.position.x = 1.8449635853
HAND_POSE_BIN_A.position.y = -1.19952466253
HAND_POSE_BIN_A.position.z = 0.547488762745

HAND_POSE_BIN_A.orientation.x = 0.385707010245
HAND_POSE_BIN_A.orientation.y = -0.650125208442
HAND_POSE_BIN_A.orientation.z = 0.350242183268
HAND_POSE_BIN_A.orientation.w = 0.553080219007

HAND_POSE_TRAY_B = Pose()
HAND_POSE_TRAY_B.position.x = 1.91710359408
HAND_POSE_TRAY_B.position.y = -0.259332117415
HAND_POSE_TRAY_B.position.z = 0.563985487433

HAND_POSE_TRAY_B.orientation.x = 0.537459932318
HAND_POSE_TRAY_B.orientation.y = -0.399998226244
HAND_POSE_TRAY_B.orientation.z = 0.660406564155
HAND_POSE_TRAY_B.orientation.w = 0.33911857834

HAND_POSE_TRAY_A = Pose()
HAND_POSE_TRAY_A.position.x = 1.44579963581
HAND_POSE_TRAY_A.position.y = -0.614457620696
HAND_POSE_TRAY_A.position.z = 0.55683081001

HAND_POSE_TRAY_A.orientation.x = 0.608976649347
HAND_POSE_TRAY_A.orientation.y = -0.732705451104
HAND_POSE_TRAY_A.orientation.z = 0.178541615939
HAND_POSE_TRAY_A.orientation.w = 0.245790670796

CONTAINER_A = Deposit()
CONTAINER_A.coord = [1.1, -0.1, -90]
CONTAINER_A.hand = HAND_POSE_CONTAINER_A

CONTAINER_B = Deposit()
CONTAINER_B.coord = [1.2, 0.1, -90]
CONTAINER_B.hand = HAND_POSE_CONTAINER_B

BIN_B = Deposit()
BIN_B.coord = [2.55,  -0.1, -60]
BIN_B.hand = HAND_POSE_BIN_B

BIN_A = Deposit()
BIN_A.coord = [2.3,  0.0, -90]
BIN_A.hand = HAND_POSE_BIN_A

TRAY_B = Deposit()
TRAY_B.coord = [1.85, 0.0, -90]
TRAY_B.hand = HAND_POSE_TRAY_B

TRAY_A = Deposit()
TRAY_A.coord = [1.6,  0.0, -90]
TRAY_A.hand = HAND_POSE_TRAY_A

POSE_TABLE1 = [-0.1, 1.3, 90]
POSE_TABLE2 = [1.1, 1.3, 90]
POSE_GROUND1 = [1.0, 0.6, 90]
POSE_GROUND2 = [1.0, 0.8, 90]
START = [-0.1, 0.6, 90]

#State class
class State():
	INIT = 0
	LOOK = 1
	GRASP = 2
	DEPOSE = 3
	END = 4


rospy.init_node("Stage2")

grasp_node = Grasping()
listener = tf.TransformListener()
yolo_object_browser = ObjectBrowserYolo('/workspace/src/robobreizh/scripts/obj_yolo.xml')

def detect_object():
	rospy.wait_for_service('/object_detection')
	detect_service = rospy.ServiceProxy('/object_detection', object_detection)
	msg = Int64(4)
	resp = ""
	start = time.time()
	end = start + 2
	try:
		resp = detect_service(msg)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

	while(resp.detected_objects.object_names[0].data == "nothing"):
		try:
			resp = detect_service(msg)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		if time.time() >= end:
			return False, False, False

	detected_obj = DetectedObj()
	detected_obj = resp.detected_objects

	ind = compute_clostest_object(detected_obj.object_posesXYZ)
	return detected_obj.objects_bb[ind], detected_obj.object_names[ind].data, detected_obj.cloud

def place_obj(hand_pose):
	grasp_node.move_to(hand_pose, "arm")

def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	sys.exit(0)

def perform_grasp(zone):
	# Get grasping pose
	rospy.loginfo("Waiting for Grasping service...")
	start = time.time()
	rospy.wait_for_service('/detect_grasps_server/detect_grasps')
	grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

	pc = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)

	msg = GraspServerRequest()
	msg.bounding_box = zone
	msg.global_cloud = pc
	resp = ""
	try:
		resp = grasp_service(msg)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

	# Grasp the object
	if resp != "":
		detected_grasp = GraspConfigList()
		detected_grasp = resp.grasp_configs.grasps
		best_grasp = GraspConfig()
		best_grasp = detected_grasp[0]

		# Move hand to the desired height
		group_name = "arm"
		group = moveit_commander.MoveGroupCommander(group_name)

		group.allow_replanning(True)
		group.set_num_planning_attempts(5)
		group.set_workspace([-3.0, -3.0, 3.0, 3.0])
		group.set_planning_time(10)

		pose_to_perform = group.get_current_pose()

		print(group.get_current_pose())
		pose_to_perform.header.frame_id = "/odom"
		pose_to_perform.pose.position.z = best_grasp.actual_pose.position.z
		print(pose_to_perform)
		#group.set_pose_target(pose_to_perform)
		group.set_joint_value_target(pose_to_perform, "hand_palm_link", True)

		plan = group.go()
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()

		if grasp_node.grasp_ground(best_grasp):
			rospy.sleep(0.5)
			move_hand(0)
			print("Grasp successful!")
		else:
			print("Grasp failed!")

		move_arm_init()

def compute_clostest_object(detected_obj):
	clostest_dist = 0.0
	indice = 0
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	print(trans)
	for i in range(len(detected_obj)):
		#Compute distance between the robot and the object
		p = grasp_node.transform_frame(detected_obj[i], "map", "head_rgbd_sensor_rgb_frame").pose
		x = p.position.x - trans[0]
		y = p.position.y - trans[1]

		dist_actual = np.sqrt(np.square(x) + np.square(y))
		print(dist_actual)

		if dist_actual > clostest_dist:

			indice = i

	return indice

def compute_clostest_object2(detected_obj):
	clostest_dist = 0.0
	indice = 0
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	print(trans)
	for i in range(len(detected_obj)):
		#Compute distance between the robot and the object
		p = grasp_node.transform_frame(detected_obj[i], "map", "odom").pose
		x = p.position.x - trans[0]
		y = p.position.y - trans[1]

		dist_actual = np.sqrt(np.square(x) + np.square(y))
		print(dist_actual)

		if dist_actual > clostest_dist:

			indice = i

	return indice

def move_arm_table(height):
	group_name = "arm"
	group = moveit_commander.MoveGroupCommander(group_name)

	group.allow_replanning(True)
	group.set_num_planning_attempts(5)
	group.set_workspace([-3.0, -3.0, 3.0, 3.0])
	group.set_planning_time(10)

	pose_to_perform = group.get_current_pose()

	pose_to_perform.header.frame_id = "/odom"
	pose_to_perform.pose.position.z = height

	#group.set_pose_target(pose_to_perform)
	group.set_joint_value_target(pose_to_perform, "hand_palm_link", True)

	plan = group.go()
	# Calling `stop()` ensures that there is no residual movement
	group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	group.clear_pose_targets()

def get_grasp_random():
	zone = [0, 100, 640, 480]
	boundingbox = BoundingBoxCoord()
	boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in zone)

	# Get grasping pose
	rospy.loginfo("Waiting for Grasping service...")
	start = time.time()
	rospy.wait_for_service('/detect_grasps_server/detect_grasps')
	grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

	pc = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)

	msg = GraspServerRequest()
	msg.bounding_box = boundingbox
	msg.global_cloud = pc
	resp = ""
	try:
		resp = grasp_service(msg)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

	if resp != "":
		detected_grasp = GraspConfigList()
		detected_grasp = resp.grasp_configs.grasps

		objects = []
		for grasp in detected_grasp:
			# pose_temp = Pose()
			# pose_temp.position.x = grasp.position.x
			# pose_temp.position.y = grasp.position.y
			# pose_temp.position.z = grasp.position.z

			objects.append(grasp.actual_pose)

		ind = compute_clostest_object2(objects)

		best_grasp = GraspConfig()
		best_grasp = detected_grasp[ind]
	print("BEST GRASP: ")
	print(best_grasp)
	return best_grasp

def go_to_place(place):
	move_base_goal(place[0], place[1], place[2])

def move_distance(dist):
	speed = 0.1
	start = time.time()
	end = start + (dist / speed)
	while time.time() < end:
		move_base_vel(speed, 0.0,0.0)

def move_angle(angle_r):
	base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
	vel_msg = Twist()
	print(angle_r)
	angular_speed = 30 * 2 * math.pi / 360

	if angle_r < 0:
		vel_msg.angular.z = abs(angular_speed)
	else:
		vel_msg.angular.z = -abs(angular_speed)

	vel_msg.linear.x = 0.0
	vel_msg.linear.y = 0.0
	t0 = rospy.Time.now().to_sec()
	current_angle = 0

	while(current_angle < abs(angle_r)):
		base_vel_pub.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)

	vel_msg.angular.z = 0
	base_vel_pub.publish(vel_msg)



def move_toward_object():
	(trans,rot) = listener.lookupTransform('/base_link', '/current', rospy.Time(0))
	

	dist = np.linalg.norm(trans)

	(trans,rot) = listener.lookupTransform('/odom', '/current', rospy.Time(0))
	theta = tf.transformations.euler_from_quaternion(rot)
	print("DIST: {}".format(dist))
	print("ANGLE: {}".format(theta))

	if dist <= 0.4:
		return 0
	(trans2,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	print(trans2)

	move_angle(theta[2])

	d_to_obj = dist - 0.4
	move_distance(d_to_obj)


def calc_dist(pose1, pose2):
	x = pose1[0] - pose2[0]
	y = pose1[1] - pose2[1]
	v = [x,y]
	dist_actual = np.sqrt(np.square(x) + np.square(y))
	return dist_actual

def get_deposit(obj_name):
	category = yolo_object_browser.getCategory(obj_name)
	dic_depo = {'Food': TRAY_A, 'Kitchen': CONTAINER_A, 'Tool': BIN_A, 'Shape': BIN_B, 'Task': BIN_A, 'Discarded': BIN_B}
	return dic_depo.get(category)

def get_grasp(detected_obj, cloud):
	rospy.loginfo("Waiting for Grasping service...")
	start = time.time()
	rospy.wait_for_service('/detect_grasps_server/detect_grasps')
	grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

	msg = GraspServerRequest()
	msg.bounding_box = detected_obj
	msg.global_cloud = cloud

	try:
		resp2 = grasp_service(msg)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		return False

	print("Time to find a good grasp: {}".format(time.time() - start))

	detected_grasp = GraspConfigList()
	detected_grasp = resp2.grasp_configs.grasps
	best_grasp = GraspConfig()
	best_grasp = detected_grasp[0]
	return best_grasp

def open_drawers():
	pass

def main():
	current_obj = RGBD()
	signal.signal(signal.SIGINT, signal_handler)

	# For testing pupropse, go to the initial position
	# repositionning()
	spawn_obj()
	state = State.INIT
	current_dep = BIN_B

	while True:

		if state == State.INIT:
			move_arm_init()
			move_hand(0)
			open_drawers()


			state = State.LOOK

		elif state == State.LOOK:
			go_to_place(POSE_GROUND1)

			move_head_tilt(-1.0)

			move_arm_init()

			bounding_box, name, cloud = detect_object()
			
			if not name:
				current_obj.set_coordinate_name("current")

				grasp = get_grasp_random()

				current_obj.set_xyz([grasp.position.x, grasp.position.y, grasp.position.z])

				move_toward_object()

				# Move hand to the desired height
				if grasp.actual_pose.position.z >= 0.7:
					move_arm_table(grasp.actual_pose.position.z)

				if grasp_node.grasp_ground(grasp):
					rospy.sleep(0.5)
					move_hand(0)
					print("Grasp successful!")
				else:
					print("Grasp failed!")

				move_arm_init()
				current_obj.set_deposit(BIN_A)

			else:
				current_obj.set_coordinate_name("current")

				grasp = get_grasp(bounding_box, cloud)

				current_obj.set_xyz([grasp.position.x, grasp.position.y, grasp.position.z])

				move_toward_object()

				# Move hand to the desired height
				if grasp.actual_pose.position.z >= 0.7:
					move_arm_table(grasp.actual_pose.position.z)

				if grasp_node.grasp_ground(grasp):
					rospy.sleep(0.5)
					move_hand(0)
					print("Grasp successful!")
				else:
					print("Grasp failed!")

				move_arm_init()
				
				depo = get_deposit(name)

				current_obj.set_deposit(depo)

			state = State.DEPOSE

		elif state == State.DEPOSE:
			go_to_place(current_obj.get_deposit().coord)

			place_obj(current_obj.get_deposit().hand)

			move_hand(1)
			
			move_arm_init()
			move_hand(0)

			state = State.LOOK

if __name__ == "__main__":
	#Start state machine
	main()

