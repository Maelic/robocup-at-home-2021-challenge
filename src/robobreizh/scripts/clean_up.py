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
from trac_ik_python.trac_ik import IK
from xml_utils import ObjectBrowserYolo
from tf import TransformListener

#State class
class State():
	NONE = 0
	SEARCH = 1
	MOVE = 2
	TAKE = 3
	DEPOSIT = 4
	END = 5

#Deposits information
class Deposit():
	name = ""
	coord = [0,0,0]
	hand = Pose()


#Object information
class Curent_object(object):
	def __init__(name, deposit, coord):
		name = name
		deposit_name = deposit
		coord = coord


def go_to_table():
	put_object("cracker_box", 0.6, 1.9, 0.5)
	put_object("mustard_bottle", 1.0, 1.8, 0.5)
	put_object("mug", 0.8, 1.7, 0.5)
	put_object("foam_brick", 1.2, 1.7, 0.5)
	rospy.init_node('main')

	
	# Move toward the table
	# look down a little
	move_head_tilt(-0.5)

	# move position
	move_arm_init()
	# move in front of the long table
	move_base_goal(1, 1, 90)

def demo_grasping():
    # put_object("cracker_box", 0.6, 1.9, 0.5)
    # put_object("mustard_bottle", 1.0, 1.8, 0.5)
    # put_object("mug", 0.8, 1.7, 0.5)
    # put_object("foam_brick", 1.2, 1.7, 0.5)

	
	# Move toward the table
	# look down a little
    move_head_tilt(-0.5)

	# move position
    move_arm_init()
	# move in front of the long table
    move_base_goal(0.6, 1, 90)
	# Look for objects
    time.sleep(1)
    #vision_module = Vision()
    #objects = vision_module.searchObject()


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
HAND_POSE_BIN_B.position.x = 2.81238017415
HAND_POSE_BIN_B.position.y = -0.253838684177
HAND_POSE_BIN_B.position.z = 0.509733161963

HAND_POSE_BIN_B.orientation.x = 0.412303506642
HAND_POSE_BIN_B.orientation.y = 0.526391918239
HAND_POSE_BIN_B.orientation.z = -0.370938176591
HAND_POSE_BIN_B.orientation.w = 0.544454991423

HAND_POSE_BIN_A = Pose()
HAND_POSE_BIN_A.position.x = 2.30236312784
HAND_POSE_BIN_A.position.y = -0.576004612216
HAND_POSE_BIN_A.position.z = 0.721292795453

HAND_POSE_BIN_A.orientation.x = 0.469319502933
HAND_POSE_BIN_A.orientation.y = -0.51863385196
HAND_POSE_BIN_A.orientation.z = 0.538487616008
HAND_POSE_BIN_A.orientation.w = 0.469882133278

HAND_POSE_TRAY_B = Pose()
HAND_POSE_TRAY_B.position.x = 1.91710359408
HAND_POSE_TRAY_B.position.y = -0.259332117415
HAND_POSE_TRAY_B.position.z = 0.563985487433

HAND_POSE_TRAY_B.orientation.x = 0.537459932318
HAND_POSE_TRAY_B.orientation.y = -0.399998226244
HAND_POSE_TRAY_B.orientation.z = 0.660406564155
HAND_POSE_TRAY_B.orientation.w = 0.33911857834

HAND_POSE_TRAY_A = Pose()
HAND_POSE_TRAY_A.position.x = 1.77153843117
HAND_POSE_TRAY_A.position.y = 0.15362261753
HAND_POSE_TRAY_A.position.z = 0.589553820905

HAND_POSE_TRAY_A.orientation.x = 0.550150405563
HAND_POSE_TRAY_A.orientation.y = -0.391995190074
HAND_POSE_TRAY_A.orientation.z = 0.61801347907
HAND_POSE_TRAY_A.orientation.w = 0.402161213824

TRAY_A = [1.5,  0.0, -90]
TRAY_B = [1.85, 0.0, -90]
BIN_A = [2.2,  0.0, -90]
BIN_B = [2.75,  0.0, -90]

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
TRAY_A.coord = [1.5,  0.0, -90]
TRAY_A.hand = HAND_POSE_TRAY_A

rospy.init_node("Manager")

grasp_node = Grasping()
listener = tf.TransformListener()

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
	end = start + 4
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

	#delete_object("gelatin_box")
	#put_object("master_chef_can", 1.3, 1.3, 0.0)
	# delete_object("mug")

	#put_object("mug", 1.2, 1.3, 0.0)
	# put_object("master_chef_can", 1.3, 1.7, 0.45)
	# put_object("sponge", 1.1, 1.8, 0.45)
	#put_object("dice", 1.5, 1.4, 0.0)

	# delete_object("spatula")
	# put_object("spatula", 1.1, 1.4, 0.0)

def move_arm_vision():
	group_name = "arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	move_group.allow_replanning(True)
	#move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
	joint_goal = move_group.get_current_joint_values()

	joints_index = {'arm_lift_joint':0, 'arm_flex_joint':1, 'arm_roll_joint':2, 'wrist_flex_joint':3, 'wrist_roll_joint':4, 'wrist_ft_sensor_frame_joint':5, 'wrist_ft_sensor_frame_inverse_joint':6, 'hand_palm_joint':7,}

	joint_goal[3] = -1.5
	joint_goal[2] = 1.5

	move_group.go(joint_goal, wait=True)
	move_group.stop()

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

def process(State):		
	yolo_object_browser = ObjectBrowserYolo('/workspace/src/robobreizh/scripts/obj_yolo.xml')

	# Look for objects
	rospy.loginfo("Waiting for Object Detection service...")
	resp = detect_object()

	while not resp:
		rospy.loginfo("No objects found, moving...")
		move_base_vel(0.1,0.0,0)
		time.sleep(1)
		resp = detect_object()

	rospy.loginfo("Objects found!")

	detected_obj = DetectedObj()
	detected_obj = resp.detected_objects

	indice = compute_clostest_object(detected_obj)

	# (trans,rot) = listener.lookupTransform('/odom', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))
	# rospy.set_param('camera_position', [trans[0], trans[1], trans[2]])

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
			
	elif State == "Table2":	
		if grasp_node.grasp_table2(best_grasp):
			move_hand(0)
			print("Grasp successful!")
		else:
			print("Grasp failed!")
			return False

	elif State == "Ground":
		if best_grasp.pre_pose.position.z >= 0.4:
			print("Grasp pose is too far on the table")
			return False
		else:
			if grasp_node.grasp_ground(best_grasp):
				move_hand(0)
				print("Grasp successful!")
			else:
				print("Grasp failed!")
				return False

	move_head_tilt(0.0)

	# Move back arm for easy navigation
	move_arm_neutral()

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


def main():
	yolo_object_browser = ObjectBrowserYolo('/workspace/src/robobreizh/scripts/obj_yolo.xml')
	rospy.init_node("Manager")
	
	# For testing pupropse, go to the initial position
	#repositioning()

	POSE_TABLE1 = [-0.1, 1.3, 90]
	POSE_TABLE2 = [1.25, 1.2, 90]
	POSE_GROUND1 = [1.2, 0.6, 90]
	POSE_GROUND2 = [1.2, 0.8, 90]

	State = "Table1"

	while True:

		if State == "Table1":

			#Step 1: Clean up Objects Table 1
			move_arm_vision()
			go_to_place(POSE_TABLE1)
			move_head_tilt(-0.6)

			# Move the arm to the table height

			group = moveit_commander.MoveGroupCommander('arm')
			joints = group.get_current_joint_values()
			print(joints)
			joints[0] = 0.3
			
			group.go(joints, wait=True)
			group.stop()
			# It is always good to clear your targets after planning with poses.
			# Note: there is no equivalent function for clear_joint_value_targets()
			group.clear_pose_targets()

			if not process(State):
				print("Grasp unsuccessful on the first Table, moving on...")
				State = "Ground"
				return_init_state()

		elif State == "Ground":

			#Step 2: Clean up Objects Ground Area
			move_arm_vision()
			go_to_place(POSE_GROUND1)

			move_head_tilt(-0.8)
			time.sleep(2)
			go_to_place(POSE_GROUND2)
			time.sleep(1)

			#move_base_vel(0.1,0.0,0)
			if not process(State):
				print("Grasp unsuccessful on the Ground, moving on...")
				State = "Table2"
				return_init_state()

		elif State == "Table2":
			#Step 3: Clean Objects Table 2
			#move_arm_vision()
			move_arm_vision()
			go_to_place(POSE_GROUND2)

			rospy.sleep(0.5)

			move_head_tilt(-0.6)

			rospy.sleep(1)

			go_to_place(POSE_TABLE2)

			rospy.sleep(0.5)

			if not process(State):
				print("Grasp unsuccessful on the Table2, exiting.")
				State = "Table2"
				return 0
		

#State machine
def start():
	#Init
	nbTidyObject = 0
	previous_state = Stat.NONE
	state = State.SEARCH

	while True:

		if state == State.SEARCH:
			#Detect and localize not deposit object
			detected_object = Vision.searchObject()

			#Next state
			previous_state = state
			state = State.MOVE

		elif state == State.MOVE:
			#Go to the destination
			if previous_state == State.SEARCH:
				Navigation.moveTo(detect_object.coord)
			elif previous_state == State.TAKE:
				Navigation.moveTo(associatedDeposit(detect_object).coord)

			#Next state
			previous_state = state
			if previous_state == State.SEARCH:
				state = State.TAKE
			elif previous_state == State.TAKE:
				state = State.DEPOSIT

		elif state == State.TAKE:
			#Take not deposit object
			Movements.takeObject(detected_object)

			#Next state
			previous_state = state
			state = State.MOVE

		elif state == State.DEPOSIT:
			#Open deposit
			Movements.openDeposit(associatedDeposit(detect_object))

			#Deposit object
			Movements.depositObject(associatedDeposit(detect_object))

			#Close deposit
			Movements.closeDeposit(associatedDeposit(detect_object))

			#Number tidy object +1
			nbTidyObject = nbTidyObject + 1

			#Next state
			previous_state = state
			if nbTidyObject == 30:
				state = State.END
			else:
				state = State.SEARCH
		
		elif state == State.END:
			#Exit 
			break

if __name__ == "__main__":
	#Start state machine
	main()



#Vision:
#object() = Vision.searchObject()

#Navigation:
#Navigation.moveTo([x,y,z])

#Movements
#Movements.takeObject(object())
#Movements.openDeposit(deposit_x())
#Movements.depositObject(deposit_x())
#Movements.closeDeposit(deposit_x())