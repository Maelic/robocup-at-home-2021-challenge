#!/usr/bin/env python
# -*- coding: utf-8 -*-

#from Vision.vision import Vision
from utils import *
import rospy
import sys
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
import json
from std_msgs.msg import String
from std_msgs.msg import Float32, Int64

from robobreizh.msg import CloudIndexed, CloudSources, GraspConfigList, DetectedObj, BoundingBoxCoord, GraspConfig, GraspServerRequest
from robobreizh.srv import detect_grasps, object_detection, grasping
from Grasping.grasping_node import Grasping
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import moveit_commander

#State class
class State():
	INIT = 0
	AWAIT_ORDERS = 1
	FIND_OBJECT = 2
	GRASP_OBJECT = 3
	DEPOSIT = 4
	END = 5

#Deposits information
class Deposit():
	name = ""
	coord = [0,0]
	handle_coord = [0,0,0]


#Object information
class Tidy_object():
	name = "object"
	deposit_name = "default"
	coord = [0,0,0]


yellow = [[22, 93, 0], [45, 255, 255]]
red = [[0,50,50], [10,255,255]]
additional_objects = {'apple': red, 'lemon':yellow}
rgbd = RGBD()

def find_HSV(obj):
	bridge = CvBridge()
	image_rgb = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_raw", Image) 
	cv_image = bridge.imgmsg_to_cv2(image_rgb, "bgr8")
	h_image = rgbd.get_h_image()

	image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	lower = np.array(additional_objects.get(obj)[0], dtype="uint8")
	upper = np.array(additional_objects.get(obj)[0], dtype="uint8")
	yellow_region = (h_image > lower) & (h_image < upper)
	#plt.imshow(yellow_region)
	mask = cv2.inRange(image, lower, upper)

	cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if len(cnts) == 2 else cnts[1]

	for c in cnts:
		x,y,w,h = cv2.boundingRect(c)
		cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)

	cv2.imshow('mask', mask)
	cv2.waitKey()


class Manager(object):
	def __init__():
		self.objects = {}
		self.current_object = Tidy_object() 
		self.deposit_1 = Deposit("deposit_1", [0,0], [0,0,0])
		self.deposit_2 = Deposit("deposit_1", [0,0], [0,0,0])
		self.deposit_3 = Deposit("deposit_1", [0,0], [0,0,0])
		self.deposit_4 = Deposit("deposit_1", [0,0], [0,0,0])
		self.deposit_5 = Deposit("deposit_1", [0,0], [0,0,0])
		self.current_state = State()

		

#Deposit coordinate
def associatedDeposit(detect_object):
	if detect_object.deposit_name == "deposit_1":
		deposit = deposit_1()
	elif detect_object.deposit_name == "deposit_2":
		deposit = deposit_2()
	elif detect_object.deposit_name == "deposit_3":
		deposit = deposit_3()
	elif detect_object.deposit_name == "deposit_4":
		deposit = deposit_4()
	elif detect_object.deposit_name == "deposit_5":
		deposit = deposit_5()

	return deposit

def move_body(x, y, z):
	p = PoseStamped()

	p.header.frame_id = "map"

	# Set target position
	p.pose.position.x = x
	p.pose.position.y = y
	p.pose.position.z = z

	# Convert Euler to Quaternion
	p.pose.orientation = quaternion_from_euler(0, 0, 0)

	# Set position and orientation target
	whole_body.set_pose_target(p)
	whole_body.go()

def take_object(x, y, z):
	move_body(x, y, z+0.2)
	time.sleep(1)
	move_body(x, y, z)

def compute_next_object(detected_obj):
	clostest = (detected_obj.object_poses[0], 0)
	indice = 0
	for obj in detected_obj.object_poses:
		indice+=1
		if obj[2] < closest[2]:
			clostest = (obj, indice)

	return clostest

START_ROOM2 = [2.6, 1.9, 90]
DETECT_OBJFIRST = [2.3, 3.5, 90]
DETECT_OBJSECOND = [2.3, 4.15, 90]

rospy.init_node("Manager_task2")

grasp_node = Grasping()
listener = tf.TransformListener()


WAIT = []

def go_to_place(place):
	move_base_goal(place[0], place[1], place[2])

def parse_message(msg):
	# Current online and competition parsing
	food_name = msg.split(" to ")[0]
	person_side = msg.split('person ')[1]
   
	# Offline version
	#food_name = msg
	#person_side =  msg.split('person ')[1]
	return food_name, person_side

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

def detect_object(object_to_find):
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
			return False, ""

	detected_obj = DetectedObj()
	detected_obj = resp.detected_objects

	for i in range(len(detected_obj.object_names)):
		if detected_obj.object_names[i].data[4:] == object_to_find:	
			print("**************** TROUVE ********************")
			return True, detected_obj.objects_bb[i]

	# If the object is not detected we choose the clostest object to pick (most likely the easiest one)
	ind = compute_clostest_object(detected_obj)
	return False, detected_obj.objects_bb[ind]

def look_for_objects(object_to_find):
	move_head_tilt(-0.3)
	go_to_place(DETECT_OBJSECOND)

	result, obj = detect_object(object_to_find)
	if result:
		return obj
	else:
		return False


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

		if grasp_node.grasp_table2(best_grasp):
			rospy.sleep(0.5)
			move_hand(0)
			print("Grasp successful!")
		else:
			print("Grasp failed!")

		move_arm_init()

def move_head_left():
	group_name = "head"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	move_group.allow_replanning(True)
	#move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
	joint_goal = move_group.get_current_joint_values()

	joints_index = {'head_pan_joint':0, 'head_tilt_joint':1}

	joint_goal[0] = 0.5
	joint_goal[1] = -0.9

	try:
		move_group.go(joint_goal, wait=True)
	except moveit_commander.MoveItCommanderException as exc:
		print("")
	move_group.stop()

def move_head_right():
	group_name = "head"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	move_group.allow_replanning(True)
	#move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
	joint_goal = move_group.get_current_joint_values()

	joints_index = {'head_pan_joint':0, 'head_tilt_joint':1}

	joint_goal[0] = 0.0
	joint_goal[1] = -1.0

	try:
		move_group.go(joint_goal, wait=True)
	except moveit_commander.MoveItCommanderException as exc:
		print("")
	move_group.stop()

def go_left():
	go_to_place([])

start2 = True

def go_shortcut():
	move_base_vel_rad(0.5,0.0,0.8)
	move_base_goal(1.65, 2.65, 180)
	move_base_goal(1.65, 3.5, 90)

def obstacle_avoidance():
	global start2

	(trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
	if trans[0] <= 1.6 and trans[1] >= 3:
		return True 
	# Check on the left
	move_head_left()
	zone = [200, 100, 640, 430]
	poseL = look_for_grasps(zone)

	if not poseL and start2:
		go_shortcut()
		start2 = False
		return "exit"
	elif not poseL:
		move_base_vel_rad(0.2,0.0,0.3)
		res = obstacle_avoidance()
		start2 = False
	else:
		start2 = False
	print(poseL)

	# Shortcut
	shortcut = True
	for pose in poseL:
		if pose.actual_pose.position.x <= 2.1 and pose.actual_pose.position.y <= 2.3:
			shortcut = False
		# pass
	if shortcut:
		go_shortcut()

		return "exit"
	poseL = poseL[0]

	# else:
	print("dist: {}".format(dist_points2([poseL.actual_pose.position.x,poseL.actual_pose.position.y], trans)))
	if dist_points2([poseL.actual_pose.position.x,poseL.actual_pose.position.y], trans) >= 0.6:
		move_base_vel_rad(0.2,0.0,0.3)
		res = obstacle_avoidance()
		if res == "exit":
			return "exit"
	move_head_right()
	zone = [300, 100, 540, 430]
	poseR = look_for_grasps(zone)
	if not poseR:
		move_base_vel_rad(0.2,0.0,-0.3)
		res = obstacle_avoidance()
		if res == "exit":
			return "exit"
	else:
		poseR = poseR[0]
	print(poseR)
	if dist_points2([poseR.actual_pose.position.x,poseR.actual_pose.position.y], trans) >= 0.6:
		move_base_vel_rad(0.2,0.0,-0.3)
		res = obstacle_avoidance()
		if res == "exit":
			return "exit"
	#Compute dist between the 2 obstacles

	dist = dist_points(poseR.actual_pose.position, poseL.actual_pose.position)
	diam = 0.7
	if dist > diam:
		center_x = (poseR.actual_pose.position.x+poseL.actual_pose.position.x)/2   
		center_y = (poseR.actual_pose.position.y+poseL.actual_pose.position.y)/2 
		move_base_goal(center_x, center_y, 90)
		move_base_vel(0.1,0.0,0.0)
		res = obstacle_avoidance()
		if res == "exit":
			return "exit"
	else:
		# Move clostest object
		obstacles = [poseL, poseR]
		ind = compute_clostest_obstacle([poseL.actual_pose, poseR.actual_pose])
		(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

		if dist_points2([obstacles[ind].actual_pose.position.x, obstacles[ind].actual_pose.position.y], trans) >= 0.4:
			move_base_vel(0.1,0.0,0.0)
			res = obstacle_avoidance()
			if res == "exit":
				return "exit"
		else:
			move_hand(0)
			grasp_node.grasp_ground(obstacles[ind])
			move_hand(1)
			move_arm_init()
			move_base_goal(2.6, 1.8, 180)
			move_base_vel(0.1,0.0,0.0)
			move_arm_neutral()

			move_hand(0)
			move_arm_init()
			move_base_goal(2.6, 1.8, 180)
			move_base_vel(0.1,0.0,0.0)
			res = obstacle_avoidance()
			if res == "exit":
				return "exit"
	rospy.sleep(1.)

def compute_clostest_obstacle(obstacles):

	clostest_dist = 0.0
	indice = 0
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	print(trans)
	for i in range(len(obstacles)):
		#Compute distance between the robot and the object
		p = grasp_node.transform_frame(obstacles[i], "map", "head_rgbd_sensor_rgb_frame").pose
		x = p.position.x - trans[0]
		y = p.position.y - trans[1]

		dist_actual = np.sqrt(np.square(x) + np.square(y))
		print(dist_actual)

		if dist_actual > clostest_dist:

			indice = i

	return indice

def dist_points2(A, B):
	x = A[0] - B[0]
	y = A[1] - B[1]

	return(np.sqrt(np.square(x) + np.square(y)))

def dist_points(A, B):
	x = A.x - B.x
	y = A.y - B.y

	return(np.sqrt(np.square(x) + np.square(y)))

def obstacle_avoidance2():
# Check on the left
	move_head_left()
	zone = [200, 100, 640, 430]
	graspL = look_for_grasps(zone)
	move_head_right()
	zone = [300, 100, 540, 430]
	graspR = look_for_grasps(zone)
	all_grasp = []
	for x in graspL:
		all_grasp.append(x.actual_pose.position)
	for x in graspR:
		all_grasp.append(x.actual_pose.position)

	# Compute all the distance between all grasp and take the longest one
	longest = 0
	points = []
	for i in range(20):
		for j in range(20):
			if dist_points(graspL[i].position, graspR):
				pass
	if poseL.position.x >= 2.1 and poseL.position.y >= 2.5:
		move_base_goal(1.65, 2.65, 90)
		pass
		

def compute_dist(sourcePose):
	(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	print(trans)
	#Compute distance between the robot and the object
	p = grasp_node.transform_frame(sourcePose, "map", "odom").pose
	x = p.position.x - trans[0]
	y = p.position.y - trans[1]

	dist_actual = np.sqrt(np.square(x) + np.square(y))
	return dist_actual

def look_for_grasps(bounding_box):
	# Look for objects
	resp = ""
	boundingbox = BoundingBoxCoord()
	boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in bounding_box)

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
		return False

	return resp.grasp_configs.grasps

#State machine
def start():
	#Init
	move_arm_init()
	move_hand(0)

	NbObjects = 0
	#previous_state = Stat.NONE
	state = State.INIT
	while True:
		if state == State.INIT:
			# Look up a little
			move_head_tilt(0.1)
			# Move position
			move_arm_init()
			# Navigate to room 2 
			go_to_place(START_ROOM2)
			#obstacle_avoidance()
			move_head_tilt(-0.7)
			grasp_node.move_arm_vision()

			# Next state
			state = State.AWAIT_ORDERS

		elif state == State.AWAIT_ORDERS:
			# Go to place with objects
			go_to_place(DETECT_OBJFIRST)
			# TODO: What if instruction is in the wrong format or if it's equal to done
			instruction  = rospy.wait_for_message('/message', String)
			print('DEBUG instruction = %s' % instruction.data)
			if (instruction.data != 'done'):
				food, person_side = parse_message(instruction.data)
				print('Bring %s to person %s' %(food, person_side))

				# Next state
				state = State.FIND_OBJECT    
				grasp_node.move_arm_vision()
				# TODO: Check objects
				obj = look_for_objects(food)
				
				# If no object is detected we perform a random grasp in the middle
				if obj == False:
					zone = [170, 100, 470, 380]
					boundingbox = BoundingBoxCoord()
					boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in zone)
					perform_grasp(boundingbox)
				else:
					perform_grasp(obj)

				# Next state
				state = State.GRASP_OBJECT

		elif state == State.GRASP_OBJECT:
			# TODO: Grasping part
			
			# Next state
			state = State.DEPOSIT

		elif state == State.DEPOSIT:
			# Navigate towards the correct person
			if person_side == 'left':
				move_base_goal(0.8, 2.86, 180) 
			else:
				move_base_goal(0.8, 3.93, 180)

			# Waiting to receive "done" message
			done_msg  = rospy.wait_for_message('/message', String)
			
			# TODO: Leave object on the ground
			move_arm_neutral()
			move_hand(1)

			move_hand(0)
			move_arm_init()

			# Next state
			NbObjects += 1
			if NbObjects >= 3: # TODO: Arbitrary number, check rulebook
				state = State.END
			else:
				state = State.AWAIT_ORDERS

		
		elif state == State.END:
			#Exit 
			break

def main():
	start()


if __name__ == "__main__":
	#Start state machine
	main()