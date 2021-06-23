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

rospy.init_node("Manager")

grasp_node = Grasping()
listener = tf.TransformListener()
yolo_object_browser = ObjectBrowserYolo('/workspace/src/robobreizh/scripts/obj_yolo.xml')

State = "Table2"

def get_deposit(category):
	dic_depo = {'Food': TRAY_A, 'Kitchen': CONTAINER_A, 'Tool': BIN_A, 'Shape': BIN_B, 'Task': BIN_A, 'Discarded': BIN_B}

	return dic_depo.get(category)

def place_obj(hand_pose):
	grasp_node.move_to(hand_pose, "arm")

def go_to_place(place):
	move_base_goal(place[0], place[1], place[2])

def detect_object():
	rospy.wait_for_service('/object_detection')
	detect_service = rospy.ServiceProxy('/object_detection', object_detection)
	msg = Int64(4)
	resp = ""
	start = time.time()
	end = start + 1
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
			return False
	return resp

def repositioning():
	move_hand(0)
	move_head_tilt(0)
	move_arm_init()
	move_base_goal(0.0,0.0,0)

def move_arm_vision():
	group_name = "arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	move_group.allow_replanning(True)
	#move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
	joint_goal = move_group.get_current_joint_values()

	joints_index = {'arm_lift_joint':0, 'arm_flex_joint':1, 'arm_roll_joint':2, 'wrist_flex_joint':3, 'wrist_roll_joint':4, 'wrist_ft_sensor_frame_joint':5, 'wrist_ft_sensor_frame_inverse_joint':6, 'hand_palm_joint':7,}

	joint_goal[3] = -1.2
	joint_goal[2] = 1.0
	joint_goal[1] = -0.1

	move_group.go(joint_goal, wait=True)
	move_group.stop()
	move_hand(0)
	
def move_arm_base():
	move_hand(0)

	joint_trajectory_publisher = rospy.Publisher(
			'/hsrb/arm_trajectory_controller/command',
			JointTrajectory, queue_size=10)

	msg = JointTrajectory()
	msg.header.stamp = rospy.Time.now()
	msg.joint_names = ["arm_flex_joint", "arm_roll_joint", "wrist_flex_joint"]
	msg.points.append(JointTrajectoryPoint())
	msg.points[0].positions = [-0.1, 1.5, -1.5]
	msg.points[0].velocities = [0.0, 0.0, 0.0]
	msg.points[0].time_from_start = rospy.Duration(2.0)

	joint_trajectory_publisher.publish(msg)
	rospy.sleep(2)

def compute_clostest_object(detected_obj):

	clostest_dist = 0.0
	indice = 0
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	print(trans)
	for i in range(len(detected_obj.object_posesXYZ)):
		#Compute distance between the robot and the object
		p = grasp_node.transform_frame(detected_obj.object_posesXYZ[i], "map", "head_rgbd_sensor_rgb_frame").pose
		x = p.position.x - trans[0]
		y = p.position.x - trans[1]

		dist_actual = np.sqrt(np.square(x) + np.square(y))
		print(dist_actual)

		if dist_actual > clostest_dist:

			indice = i

	return indice

def calcul_distance(detected_obj):
	return calc_dist(detected_obj.object_posesXYZ[0], "head_rgbd_sensor_rgb_frame")

def calc_dist(pose, frame):
	obj_pose = grasp_node.transform_frame(pose, "map", frame).pose
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	x = obj_pose.position.x - trans[0]
	y = obj_pose.position.y - trans[1]
	v = [x,y]
	dist_actual = np.sqrt(np.square(x) + np.square(y))
	print(dist_actual)
	return dist_actual

def move_distance(dist):
	speed = 0.1
	start = time.time()
	end = start + (dist / speed)
	while time.time() < end:
		move_base_vel(speed, 0.0,0.0)

def move_object_on_the_way(stop):
	bounding_box = [170, 300, 470, 480]
	# Look for objects
	resp = ""
	boundingbox = BoundingBoxCoord()
	boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in bounding_box)

	while True:
		move_head_tilt(-1.0)

		(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		print(trans)
		if trans[1] >= stop:
			return True

		rospy.loginfo("Waiting for Object Detection service...")
		resp_obj = detect_object()
		rospy.sleep(1)
		if not resp_obj:
			# Get grasping pose
			rospy.loginfo("Waiting for Grasping service...")
			start = time.time()
			rospy.wait_for_service('/detect_grasps_server/detect_grasps')
			grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

			pc = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)

			msg = GraspServerRequest()
			msg.bounding_box = boundingbox
			msg.global_cloud = pc

			try:
				resp = grasp_service(msg)
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))

			# Grasp to the bin

			if resp != "":
				detected_grasp = GraspConfigList()
				detected_grasp = resp.grasp_configs.grasps
				best_grasp = GraspConfig()
				best_grasp = detected_grasp[0]

				if best_grasp.pre_pose.position.z >= 0.7:
					print("Grasp pose is too far on the table")
					#return False
				elif calc_dist(best_grasp.pre_pose, "odom") >= 0.5:
					print("Grasp pose is too far")
					move_distance(0.1)
					rospy.sleep(1.)
				else:
					if grasp_node.grasp_ground(best_grasp):
						move_hand(0)
						print("Grasp successful!")
					else:
						print("Grasp failed!")
					move_head_tilt(0.0)

					# Move back arm for easy navigation
					move_arm_neutral()
					#go_to_place([0,0,0])
					go_to_place(BIN_A.coord)
					rospy.sleep(1.)

					place_obj(BIN_A.hand)
					rospy.sleep(1.)

					move_hand(1)
					rospy.sleep(1.)
					
					move_arm_init()
					move_hand(0)
					#go_to_place([0,0,0])
					go_to_place([trans[0], trans[1], 90])
			else:
				rospy.loginfo("No objects found, moving...")
				move_distance(0.1)
				rospy.sleep(1.)
		else:

			rospy.loginfo("Objects found!")

			detected_obj = DetectedObj()
			detected_obj = resp_obj.detected_objects

			if calcul_distance(detected_obj) < 0.7:

				indice = compute_clostest_object(detected_obj)
				# Get grasping pose
				rospy.loginfo("Waiting for Grasping service...")
				start = time.time()
				rospy.wait_for_service('/detect_grasps_server/detect_grasps')
				grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

				msg = GraspServerRequest()
				msg.bounding_box = detected_obj.objects_bb[indice]
				msg.global_cloud = detected_obj.cloud

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

				if grasp_node.grasp_ground(best_grasp):
					move_hand(0)
					print("Grasp successful!")
				else:
					print("Grasp failed!")
					return False
				move_head_tilt(0.0)

				category = yolo_object_browser.getCategory(detected_obj.object_names[indice].data)
				print("Object category: {}".format(category))

				depo = get_deposit(category)

				go_to_place(depo.coord)

				place_obj(depo.hand)

				move_hand(1)
				
				move_arm_init()
				move_hand(0)
				#go_to_place([0,0,0])
				go_to_place([trans[0], trans[1], 90])
			else:
				rospy.loginfo("No objects found, moving...")
				move_distance(0.1)
				rospy.sleep(1.)

def process(State):		
	yolo_object_browser = ObjectBrowserYolo('/workspace/src/robobreizh/scripts/obj_yolo.xml')

	# Look for objects
	rospy.loginfo("Waiting for Object Detection service...")
	resp = detect_object()
	turn = -20

	while not resp:
		rospy.loginfo("No objects found, moving...")
		move_base_vel(0.0,0.0,turn)
		time.sleep(1)
		resp = detect_object()
		turn = 40 if (turn < 0) else -40


	rospy.loginfo("Objects found!")

	detected_obj = DetectedObj()
	detected_obj = resp.detected_objects

	indice = compute_clostest_object(detected_obj)

	# Get grasping pose
	rospy.loginfo("Waiting for Grasping service...")
	start = time.time()
	rospy.wait_for_service('/detect_grasps_server/detect_grasps')
	grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

	msg = GraspServerRequest()
	msg.bounding_box = detected_obj.objects_bb[indice]
	msg.global_cloud = detected_obj.cloud

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

	if State == "Ground":
		if best_grasp.pre_pose.position.y > 1:
			move_distance(0.65)
			time.sleep(1)
			resp = detect_object()
			turn = -45
			while not resp:
				turn = 45 if (turn == -45) else -45
				rospy.loginfo("No objects found, moving around...")
				move_base_vel(0.0,0.0,turn)
				time.sleep(1)
				resp = detect_object()

			rospy.loginfo("Objects found!")

			detected_obj = DetectedObj()
			detected_obj = resp.detected_objects

			indice = compute_clostest_object(detected_obj)

			# Get grasping pose
			rospy.loginfo("Waiting for Grasping service...")
			start = time.time()
			rospy.wait_for_service('/detect_grasps_server/detect_grasps')
			grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

			msg = GraspServerRequest()
			msg.bounding_box = detected_obj.objects_bb[indice]
			msg.global_cloud = detected_obj.cloud

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



	if State == "Table1":
		if grasp_node.grasp_table1(best_grasp):
			move_hand(0)
			print("Grasp successful!")
		else:
			print("Grasp failed!")
			return False
		move_head_tilt(0.0)

		# Move back arm for easy navigation
		move_arm_neutral()


	elif State == "Table2":	
		if grasp_node.grasp_table2(best_grasp):
			move_hand(0)
			print("Grasp successful!")
		else:
			print("Grasp failed!")
			return False
		move_head_tilt(0.0)

		# Move back arm for easy navigation
		move_arm_neutral()
		go_to_place(POSE_GROUND1)


	category = yolo_object_browser.getCategory(detected_obj.object_names[indice].data)
	print("Object category: {}".format(category))

	depo = get_deposit(category)

	go_to_place(depo.coord)

	place_obj(depo.hand)

	move_hand(1)
	rospy.sleep(0.5)
	
	move_arm_init()
	move_hand(0)
	return True

def return_init_state():
	move_hand(0)
	move_head_tilt(0.0)
	move_arm_init()

def timer_thread(start):
	end = start + 299
	while True:
		current = rospy.get_time()
		if current >= end:
			print(current)
			rospy.signal_shutdown("Timeout reached")
		rospy.sleep(1.)

def main():
	rospy.init_node("Manager")
	# For testing pupropse, go to the initial position
	spawn_obj()
	repositioning()
	start = rospy.get_time()
	print(start)
	t = threading.Thread(target=timer_thread, args=(start,))
	t.start()
	State = "Table2"
	move_arm_init()
	#move_distance(1)
	#go_to_place(CONTAINER_A.coord)
	#go_to_place(TRAY_A.coord)
	POSE_TABLE1 = [-0.1, 1.3, 90]
	POSE_TABLE2 = [1.1, 1.3, 90]
	POSE_GROUND1 = [1.1, 0.5, 90]
	POSE_GROUND2 = [1.1, 0.8, 90]
	START = [-0.1, 0.5, 90]
	POSE_GROUND12 = [0.8, 0.5, 90]
	POSE_GROUND22 = [0.8, 0.8, 90]

	#return_init_state()
	
	while True:

		if State == "Table1":
			move_arm_init()
			go_to_place(START)
			move_object_on_the_way(1.1)
			rospy.sleep(1.)
			#Step 1: Clean up Objects Table 1
			move_arm_init()
			go_to_place(POSE_TABLE1)
			move_head_tilt(0.0)
			rospy.sleep(1.)

			# Move the arm to the table height

			group = moveit_commander.MoveGroupCommander('arm')
			joints = group.get_current_joint_values()
			print(joints)
			joints[0] = 0.3
			try:
				group.go(joints, wait=True)
			except moveit_commander.MoveItCommanderException as exc:
				print("")
			
			group.stop()
			# It is always good to clear your targets after planning with poses.
			# Note: there is no equivalent function for clear_joint_value_targets()
			group.clear_pose_targets()
			move_head_tilt(-0.6)

			if not process(State):
				print("Grasp unsuccessful on the first Table, moving on...")
				State = "Table2"
				return_init_state()

		elif State == "Table2":
			#Step 2: Clean Objects Table 2
			move_arm_init()
			go_to_place(POSE_GROUND1)
			move_object_on_the_way(1.1)

			rospy.sleep(0.5)
			go_to_place(POSE_GROUND12)
			move_object_on_the_way(1.1)

			rospy.sleep(1.)

			move_head_tilt(-0.8)

			go_to_place(POSE_TABLE2)

			rospy.sleep(0.5)

			# Move the arm to the table height

			group = moveit_commander.MoveGroupCommander('arm')
			joints = group.get_current_joint_values()
			print(joints)
			joints[0] = 0.3
			try:
				group.go(joints, wait=True)
			except moveit_commander.MoveItCommanderException as exc:
				print("")
			group.stop()

			group.clear_pose_targets()

			if not process(State):
				print("Grasp unsuccessful on the Table2, exiting.")
				State = "Table2"
		

if __name__ == "__main__":
	#Start state machine
	main()

