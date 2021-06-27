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
from drawer_opener import DrawerOpener
import random

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

POSE_TABLE1 = [-0.1, 0.8, 90]
POSE_TABLE2 = [1.1, 1.3, 90]
POSE_GROUND1 = [1.0, 0.6, 90]
POSE_GROUND2 = [1.0, 0.8, 0.0]
START = [-0.1, 0.6, 90]

#State class
class State():
	INIT = 0
	LOOK = 1
	GRASP = 2
	DEPOSE = 3
	GRASP = 4
	END = 5


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
			clostest_dist = dist_actual

			indice = i

	return indice

def compute_clostest_object2(detected_obj):
	clostest_dist = 0.0
	indice = 0
	too_far = True
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
			clostest_dist = dist_actual
			indice = i
	return indice

def move_arm_vision():
	group_name = "arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	move_group.allow_replanning(True)
	#move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
	joint_goal = move_group.get_current_joint_values()

	joints_index = {'arm_lift_joint':0, 'arm_flex_joint':1, 'arm_roll_joint':2, 'wrist_flex_joint':3, 'wrist_roll_joint':4, 'wrist_ft_sensor_frame_joint':5, 'wrist_ft_sensor_frame_inverse_joint':6, 'hand_palm_joint':7,}

	joint_goal[0] = 0.5
	#joint_goal[2] = 1.4
	#joint_goal[1] = -0.1

	try:
		move_group.go(joint_goal, wait=True)
	except moveit_commander.MoveItCommanderException as exc:
		print("")
	move_group.stop()
	move_hand(0)

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
	zone = [220, 200, 420, 380]
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
	else:
		return False
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

def transform_frame(pose, frame_dest, frame_source):
	tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
	tf2_listener = tf2_ros.TransformListener(tf2_buffer)

	transform = tf2_buffer.lookup_transform(frame_dest,
								   frame_source, #source frame
								   rospy.Time(0), #get the tf at first available time
								   rospy.Duration(2.0)) #wait for 1 second

	pose_stamped_to_transform = PoseStamped()
	pose_stamped_to_transform.pose.position = pose.position
	pose_stamped_to_transform.pose.orientation = pose.orientation

	return(tf2_geometry_msgs.do_transform_pose(pose_stamped_to_transform, transform).pose)
		

def move_toward_object(grasp):
	(trans,rot) = listener.lookupTransform('/base_link', '/current', rospy.Time(0))

	dist = np.linalg.norm(trans)
	transformed_pose = transform_frame(grasp.actual_pose, "map", "odom")

	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

	X = (trans[0] + transformed_pose.position.x) / 2
	Y = (trans[1] + transformed_pose.position.y) / 2

	theta = tf.transformations.euler_from_quaternion(rot)
	print("DIST: {}".format(dist))
	print("ANGLE: {}".format(theta))

	(trans,rot) = listener.lookupTransform('/odom', '/current', rospy.Time(0))
	(trans2,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

	current_ang_rad = whole_body.get_current_joint_values()[2]
	print(current_ang_rad)
	angle_diff_rad = current_ang_rad + theta[2]
	angle_diff_deg = math.degrees(angle_diff_rad)

	# d = 0.4 / dist
	# X = trans2[0] + d * (trans[0] - trans2[0])
	# Y = trans2[1] + d * (trans[1] - trans2[1])



	print("COORD: {}".format([X,Y,angle_diff_deg]))

	if dist <= 0.5:
		X = trans[0]
		Y = trans[1]
		return 0

	move_base_goal(X, Y, angle_diff_deg)

def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return np.rad2deg((ang1 - ang2) % (2 * np.pi))


def move_toward_object2(grasp):
	(trans,rot) = listener.lookupTransform('/base_link', '/current', rospy.Time(0))

	x1 = grasp.actual_pose.position.x
	y1 = grasp.actual_pose.position.y
	p = whole_body.get_current_pose().pose
	x2 = p.position.x
	y2 = p.position.y
	p2 = [x1, y1]
	p1 = [x2, y2]

	theta = angle_between(p2, p1)

	current_ang_rad = euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]
	#current_ang_rad = whole_body.get_current_joint_values()[2]
	dist2 = np.sqrt(((x1 - x2)**2 + (y1 - y2)**2))
	print("dist2 :{}".format(dist2))
	print("current angle rad {}".format(current_ang_rad))

	#angle_diff_deg = math.degrees(angle_diff_rad)

	# print("THETA: {}".format(theta))
	# # theta = theta - 5
	# if theta >= 180:
	# 	theta = theta - 360
	#move_ang(-theta)

	rospy.sleep(.5)
	if dist2 >= 0.4:
		move_distance(dist2-0.4)
	rospy.sleep(.5)



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

def repositioning():
	move_hand(0)
	move_head_tilt(0)
	move_arm_init()
	move_base_goal(0.0,0.0,0)

def replace_robot_angular(req_angle):
	# TODO: Only works with negative values, but sufficient for RoboCup
	current_ang_rad = whole_body.get_current_joint_values()[2]
	print(current_ang_rad)
	angle_diff_rad = current_ang_rad - req_angle
	angle_diff_deg = math.degrees(angle_diff_rad)
	print(angle_diff_deg)
	move_ang(angle_diff_deg)
	
def move_ang(angle):
	speed_compute = 10
	start = time.time()
	end = start + (abs(angle) / speed_compute)
	print (end-start)
	
	if abs(angle) <= 5:
		return 0
	
	if angle <= 0:
		speed = speed_compute
	else:
		speed = -speed_compute
	
	print(speed)
		
	while time.time() < end:
		move_base_vel(0.0, 0.0, speed)
		
def main():
	current_obj = RGBD()
	signal.signal(signal.SIGINT, signal_handler)

	# For testing pupropse, go to the initial position
	#repositioning()
	#spawn_obj()
	state = State.INIT
	count_object = 0
	turn = 0
	while True:
		if count_object == 5:
			global POSE_GROUND1 
			POSE_GROUND1 = POSE_TABLE1
			go_to_place(POSE_GROUND1)

		if state == State.INIT:
			move_arm_init()
			move_hand(0)
			opener = DrawerOpener()

			#opener.open_drawers()

			go_to_place(POSE_GROUND1)

			state = State.LOOK

		elif state == State.LOOK:

			move_head_tilt(-1.0)
			if turn == 1:
				move_base_vel(0.0,0.0,0.3)
			elif turn == 2: 
				move_base_vel(0.0,0.0,-0.3)

			move_arm_init()
			move_arm_vision()
			bounding_box, name, cloud = detect_object()
			
			if not name:
				current_obj.set_coordinate_name("current")

				grasp = get_grasp_random()

				if not grasp:
					move_base_vel(0.1, 0.0, 0.0)
					state = State.LOOK
				else:
					if grasp.actual_pose.position.y >= 1.4:
						count_object = count_object + 1
						state = State.LOOK
					else:
						current_obj.set_xyz([grasp.position.x, grasp.position.y, grasp.position.z])

						move_toward_object2(grasp)

						current_obj.set_grasp(grasp)
						current_obj.set_deposit(BIN_A)

						state = State.GRASP

			else:
				current_obj.set_coordinate_name("current")

				grasp = get_grasp(bounding_box, cloud)

				if not grasp:
					move_base_vel(0.1, 0.0, 0.0)
					state = State.LOOK
				else:
					if grasp.actual_pose.position.y >= 1.4:
						count_object = count_object + 1
						current_obj.set_coordinate_name("current")

						grasp = get_grasp_random()

						if not grasp:
							move_base_vel(0.1, 0.0, 0.0)
							state = State.LOOK
						else:
							if grasp.actual_pose.position.y >= 1.5:
								count_object = count_object + 1
								state = State.LOOK
							else:
								current_obj.set_xyz([grasp.position.x, grasp.position.y, grasp.position.z])

								move_toward_object2(grasp)

								current_obj.set_grasp(grasp)
								current_obj.set_deposit(BIN_A)

								state = State.GRASP

						state = State.LOOK
					else:
						current_obj.set_xyz([grasp.position.x, grasp.position.y, grasp.position.z])

						move_toward_object2(grasp)

						depo = get_deposit(name)
						current_obj.set_deposit(depo)
						current_obj.set_grasp(grasp)

						state = State.GRASP

		elif state == State.GRASP:

			current_grasp = current_obj.get_grasp()

			# Potential collision with the table, aborting
			if current_grasp.actual_pose.position.y >= 1.1:
				move_base_vel_rad(0.0, 0.0, 0.3)
				state = State.LOOK

			# Move hand to the desired height
			if current_grasp.actual_pose.position.z >= 0.7:
				move_arm_table(current_grasp.actual_pose.position.z)

			if grasp_node.grasp_ground(current_grasp):
				rospy.sleep(0.5)
				move_hand(0)
				print("Grasp successful!")
			else:
				print("Grasp failed!")

			move_arm_init()

			state = State.DEPOSE

		elif state == State.DEPOSE:
			go_to_place(POSE_GROUND2)

			go_to_place(current_obj.get_deposit().coord)

			place_obj(current_obj.get_deposit().hand)

			move_hand(1)
			
			move_arm_init()
			move_hand(0)

			go_to_place(POSE_GROUND1)
			
			count_object = count_object + 1

			state = State.LOOK
			if turn == 0:
				turn = 1
			elif turn == 1:
				turn = 2
			elif turn == 2:
				turn = 0

if __name__ == "__main__":
	#Start state machine
	main()

