#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import os

import rospy

from sensor_msgs.msg import PointCloud2, Image, PointCloud, PointField, JointState
from geometry_msgs.msg import Point32, Point, PoseStamped, Pose, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import time
from std_msgs.msg import String, Header, Int64, Bool
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import tf
import message_filters
from robobreizh.msg import CloudIndexed, CloudSources, GraspConfigList, BoundingBoxCoord, GraspConfig
from robobreizh.srv import detect_grasps, object_detection, grasping
import moveit_commander
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from utils import *
from scipy.spatial.transform import Rotation as R
import sys
from tf import TransformListener
import tf2_ros
import tf2_geometry_msgs
import threading
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Grasping():
	def __init__(self):
		#rospy.init_node('GraspingNode', anonymous=False)

		self.object_pose = [0,0,0,0]
		self.cloud_points = []

		# self.object_pc_pub = rospy.Publisher("crop_pointcloud", PointCloud2, queue_size=10)
		# self.cloud_index_pub = rospy.Publisher("gpd_cloud_indexed", CloudIndexed, queue_size=10)

		# MOVEIT CONFIG
		self.listener = tf.TransformListener()

		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.whole_body = moveit_commander.MoveGroupCommander("whole_body_light")
		self.scene = moveit_commander.PlanningSceneInterface()
		self.arm = moveit_commander.MoveGroupCommander('arm')
		self.object_coord = ""
		self.cam_frame = "head_rgbd_sensor_rgb_frame"
	# 	t = threading.Thread(target=self.publish_frame)
	# 	t.start()

	def publish_frame(self):
		rate = rospy.Rate(10)
		while(not rospy.is_shutdown()):
			if self.object_coord != "":
				self.tb = tf.TransformBroadcaster()
				pose = [self.object_coord.position.x, self.object_coord.position.y, self.object_coord.position.z]
				orientation = [self.object_coord.orientation.x, self.object_coord.orientation.y, self.object_coord.orientation.z, self.object_coord.orientation.w]
				self.tb.sendTransform(pose, orientation,  rospy.Time.now(), "current_grasp", self.cam_frame)
				rate.sleep()

	def continuous_node(self):
		# PUBLISHERS
		self.grasp_pub = rospy.Publisher("best_grasp", GraspConfig, queue_size=10)

		# SUBSCRIBERS
		self.detected_obj_sub = message_filters.Subscriber('/detected_object', BoundingBoxCoord)
		self.pc_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
		
		while not rospy.is_shutdown():

			ts = message_filters.ApproximateTimeSynchronizer([self.detected_obj_sub, self.pc_sub], 10)
			ts.registerCallback(self.callback)

			rospy.spin()


	def callback(self, detected_obj_sub, pc_sub):
		obj = detected_obj_sub
		msg = self.create_cloud_indexed(obj, pc_sub)

		rospy.wait_for_service('/detect_grasps_server/detect_grasps')
		grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)

		try:
			resp = grasp_service(msg)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

		best_res = GraspConfig()
		best_res = resp.grasp_configs.grasps[0]
		self.grasp_pub.publish(best_res)

	def service_node(self):
		s = rospy.Service('grasping', grasping, self.handle_grasping)
		rospy.loginfo("Grasping Node: Waiting for Request...")
		rospy.spin()

	def handle_grasping(self, req):
		pose = req.best_grasp
		self.object_coord = pose
		res = Bool()
		res.data = self.grasp_gound(pose)
		res.data = self.grasp_table1(pose)

		return res


	def move_base_goal2(self, x, y):
		navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		goal = MoveBaseGoal()

		goal.target_pose.header.frame_id = "map"

		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y

		goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, 90)

		navclient.send_goal(goal)
		navclient.wait_for_result()
		state = navclient.get_state()
		return True if state == 3 else False

	def grasp_ground(self, grasp_pose):
		time.sleep(1)

		# Move toward the object
		# dist = 0.6

		# obj_pose = self.transform_frame(grasp_pose.actual_pose, "map", "odom").pose
		# #obj_pose = grasp_pose.pre_pose
		# print(obj_pose)
		# (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
		# print(trans)
		# x = obj_pose.position.x - trans[0]
		# y = obj_pose.position.y - trans[1]
		# v = [x,y]
		# dist_actual = np.sqrt(np.square(x) + np.square(y))
		# print(dist_actual)
		# if dist_actual > dist:
		# 	u = np.array(v/np.linalg.norm(v))
		# 	print(u)
		# 	dest = []
		# 	dest.append(obj_pose.position.x - dist*u[0])
		# 	dest.append(obj_pose.position.x - dist*u[1])

		# 	print(dest)
		# 	move_head_tilt(0.0)
		# 	move_base_goal(dest[0], dest[1], 90)

		move_hand(1)

		#self.visualize_pose(grasp_pose.pre_pose)
		self.move_to(grasp_pose.pre_pose, 'arm')

		#time.sleep(1)
		self.move_to(grasp_pose.actual_pose, 'arm')

		#move_hand(0)

		return True

	def grasp_table2(self, grasp_pose):
				
		move_hand(1)

		self.move_to(grasp_pose.pre_pose, 'arm')

		self.move_to(grasp_pose.actual_pose, 'arm')

		return True

	def grasp_table1(self, grasp_pose):
		move_hand(1)

		self.move_to(grasp_pose.pre_pose, 'arm')

		self.move_to(grasp_pose.actual_pose, 'arm')

		return True

	def transform_frame(self, pose, frame_dest, frame_source):
		tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		tf2_listener = tf2_ros.TransformListener(tf2_buffer)

		transform = tf2_buffer.lookup_transform(frame_dest,
									   frame_source, #source frame
									   rospy.Time(0), #get the tf at first available time
									   rospy.Duration(2.0)) #wait for 1 second

		pose_stamped_to_transform = PoseStamped()
		pose_stamped_to_transform.pose.position = pose.position
		pose_stamped_to_transform.pose.orientation = pose.orientation

		return(tf2_geometry_msgs.do_transform_pose(pose_stamped_to_transform, transform))
		
	def move_to(self, pose, target):
		group = moveit_commander.MoveGroupCommander(target)
		group.allow_replanning(True)
		group.set_num_planning_attempts(5)
		group.set_workspace([-3.0, -3.0, 3.0, 3.0])
		group.set_planning_time(10)
		pose_to_perform = group.get_current_pose()

		print(group.get_current_pose())
		pose_to_perform.header.frame_id = "/odom"
		pose_to_perform.pose.position = pose.position
		pose_to_perform.pose.orientation = pose.orientation
		print(pose_to_perform)
		#group.set_pose_target(pose_to_perform)
		group.set_joint_value_target(pose_to_perform, "hand_palm_link", True)

		plan = group.go()
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()


	def visualize_pose(self, pose):
		
		pub = rospy.Publisher("/visualize_pose", PointCloud2, queue_size=10)
		points = []
		points.append([pose.position.x, pose.position.y, pose.position.z])

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
		  PointField('y', 4, PointField.FLOAT32, 1),
		  PointField('z', 8, PointField.FLOAT32, 1),
		  ]
		header = Header()
		header.frame_id = "odom"
		header.stamp = rospy.Time.now()
		poses_msg = pc2.create_cloud(header, fields, points)

		pub.publish(poses_msg)

	def trac_IK(self, grasp_pose):
		urdfstring = ''.join(open('/opt/ros/melodic/share/hsrb_description/robots/hsrb4s.urdf', 'r').readlines())
		#urdfstring = rospy.get_param('/robot_description')
		joint_values = rospy.wait_for_message("/hsrb/joint_states", JointState)
		joint_pose = []
		ik = IK("arm_lift_link", "hand_palm_link", urdf_string=urdfstring)

		for name in ik.joint_names:
			for i in range(0, len(joint_values.name)):
				if joint_values.name[i] == name:
					joint_pose.append(joint_values.position[i])

		joint_pose.append(0.0)
		
		print("TRAC IK")
		print(ik.get_ik(joint_pose, grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w))
	
# 	seed_state = [0.0] * ik_solver.number_of_joints

# ik_solver.get_ik(seed_state,
#                 0.45, 0.1, 0.3,  # X, Y, Z
#                 0.0, 0.0, 0.0, 1.0)  # QX, QY, QZ, QW

	def move_pose_joint(self, pose, joint):
		group_name = "arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)
		move_group.allow_replanning(True)
		move_group.set_workspace([-2.0, -2.0, 2.0, 2.0])
		joint_goal = move_group.get_current_joint_values()
		
		# 0 = arm_lift_joint
		# 1 = arm_flex_joint
		# 2 = arm_roll_joint
		# 3 = wrist_flex_joint
		# 4 = wrist_roll_joint
		# 5 = wrist_ft_sensor_frame_joint
		# 6 = wrist_ft_sensor_frame_inverse_joint
		# 7 = hand_palm_joint

		joints_index = {'arm_lift_joint':0, 'arm_flex_joint':1, 'arm_roll_joint':2, 'wrist_flex_joint':3, 'wrist_roll_joint':4, 'wrist_ft_sensor_frame_joint':5, 'wrist_ft_sensor_frame_inverse_joint':6, 'hand_palm_joint':7,}

		joint_goal[joints_index.get(joint)] = joint_goal[joints_index.get(joint)] + pose

		move_group.go(joint_goal, wait=True)
		move_group.stop()
		

	def move_base(self, angle):
		PI = 3.1415926535897
		velocity_publisher = rospy.Publisher("/hsrb/cmd_vel_controller/cmd_vel", Twist, queue_size=10)
		vel_msg = Twist()
		
		#Converting from angles to radians
		speed = 10 # degree/s
		angular_speed = speed*2*PI/360
   
		#We wont use linear components
		vel_msg.linear.x=0
		vel_msg.linear.y=0
		vel_msg.linear.z=0
		vel_msg.angular.y = 0
 
		# Checking if our movement is CW or CCW
		if angle < 0:
			vel_msg.angular.z = -abs(angular_speed)
		else:
			vel_msg.angular.z = abs(angular_speed)

		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
   
		while(current_angle < angle):
			velocity_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1-t0)

		#Forcing our robot to stop
		vel_msg.angular.z = 0
		velocity_publisher.publish(vel_msg)

	def compute_inverse_kinematics(self, grasp_pose):
		#self.trac_IK(grasp_pose)
		group_joints = ["odom_r", "arm_lift_joint", "arm_flex_joint", "arm_roll_joint", ]
		tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		tf2_listener = tf2_ros.TransformListener(tf2_buffer)

		transform = tf2_buffer.lookup_transform("base_link",
									   "head_rgbd_sensor_rgb_frame", #source frame
									   rospy.Time(0), #get the tf at first available time
									   rospy.Duration(2.0)) #wait for 1 second

		pose_stamped_to_transform = PoseStamped()
		pose_stamped_to_transform.pose.position = grasp_pose.position
		pose_stamped_to_transform.pose.orientation = grasp_pose.orientation

		#grasp_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped_to_transform, transform)

		pose_target = [grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z] # End effector position = Te.
		orient_target = [grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w] # End effector orientation as a quaternion.

		# R0_g = end-effector(gripper) rotation transformation(4X4)
		R0_g = tf.transformations.quaternion_matrix(orient_target)
		# D0_g = end-effector(gripper) translation transformation(3X3)
		D0_g = tf.transformations.translation_matrix(pose_target)
		print(D0_g)
		
		origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

		(trans,rot) = self.listener.lookupTransform('/arm_lift_link', '/odom_xr_link', rospy.Time(0))
		dlift = np.linalg.norm(trans)

		(trans,rot) = self.listener.lookupTransform('/hand_palm_link', '/wrist_flex_link', rospy.Time(0))
		dwrist = np.linalg.norm(trans)

		(trans,rot) = self.listener.lookupTransform('/arm_flex_link', '/wrist_flex_link', rospy.Time(0))
		darm = np.linalg.norm(trans)


		approach = [grasp_pose.approach.x, grasp_pose.approach.y, grasp_pose.approach.z]

		binormal = [grasp_pose.binormal.x, grasp_pose.binormal.y, grasp_pose.binormal.z]

		hand_axis = [grasp_pose.axis.x, grasp_pose.axis.y, grasp_pose.axis.z] 

		R = [approach, binormal, hand_axis]

		# 1. Compute the base yaw angle 
		#(trans,rot) = self.listener.lookupTransform('/base_link', '/hand_palm_link', rospy.Time(0))
		#trans = tf.transformations.translation_matrix([0.0,0.0,-dwrist])

		(trans,rot) = self.listener.lookupTransform('/base_link', '/current_grasp', rospy.Time(0))
		print(R)
		wristX = trans[0] - (dwrist*R[0][2])
		wristY = trans[1] - (dwrist*R[1][2])
		wristZ = trans[2] - (dwrist*R[2][2])
		wrist_pos = [wristX,wristY,wristZ]

		print("wrist_pos: {}".format(wrist_pos))
		base_yaw = np.arctan2(wrist_pos[1], wrist_pos[0])
		print("base_yaw: {}".format(base_yaw))

		tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		tf2_listener = tf2_ros.TransformListener(tf2_buffer)

		transform = tf2_buffer.lookup_transform("current_grasp",
									   "base_link", #source frame
									   rospy.Time(0), #get the tf at first available time
									   rospy.Duration(2.0)) #wait for 1 second

		pose_stamped_to_transform = PoseStamped()
		pose_stamped_to_transform.pose.position.x = wristX
		pose_stamped_to_transform.pose.position.y = wristY
		pose_stamped_to_transform.pose.position.z = wristZ

		wrist_pos = tf2_geometry_msgs.do_transform_pose(pose_stamped_to_transform, transform).pose.position
		print(wrist_pos)
		# 2. Compute torso lift height
		lift_heigth = wrist_pos.z - np.sqrt(np.square(darm) - np.square(wrist_pos.x) - np.square(wrist_pos.y))

		#lift_heigth = wrist_pos[2] - np.sqrt((darm**2) - (wrist_pos[0]**2) - ((wrist_pos[1])**2))
		print("lift_heigth: {}".format(lift_heigth))

		# 3. Compute shoulder pitch angle
		shoulder_angle = np.arcsin(np.sqrt(np.square(wrist_pos.x) + np.square(wrist_pos.y))/darm)
		print("shoulder_angle: {}".format(shoulder_angle))

		# 4. Compute the wirst joint angles
		trans_arm = tf.transformations.translation_matrix([0.0, 0.0, -darm])
		trans_lift = tf.transformations.translation_matrix([0.0, 0.0, -dlift])
		trans_wrist = tf.transformations.translation_matrix([0.0, 0.0, -dwrist])
		
		Ry = tf.transformations.rotation_matrix(-shoulder_angle, yaxis)
		print(Ry)
		Rz = tf.transformations.rotation_matrix(-base_yaw, zaxis)
		print(Rz)
		R = np.dot(np.dot(np.dot(np.dot(trans_arm, Ry), trans_lift), Rz), trans)
		
		print(R)
		roll_arm = np.arctan2(R[1][2], R[0][2])
		pitch_arm = np.arccos(R[2][2])
		roll_gripper = np.arctan2(R[2][1], -R[2][0])
		print("roll_arm: {}".format(roll_arm))
		print("pitch_arm: {}".format(pitch_arm))
		print("roll_gripper: {}".format(roll_gripper))

		#arm_pub = rospy.Publisher()arm_lift_joint

		return base_yaw, lift_heigth, shoulder_angle, roll_arm, pitch_arm, roll_gripper


		
	def grasp_object2(self, grasp_pose):
		self.object_coord = grasp_pose

		approach = [grasp_pose.approach.x, grasp_pose.approach.y, grasp_pose.approach.z]

		binormal = [grasp_pose.binormal.x, grasp_pose.binormal.y, grasp_pose.binormal.z]

		hand_axis = [grasp_pose.axis.x, grasp_pose.axis.y, grasp_pose.axis.z] 

		aperture = grasp_pose.width

		orientation = grasp_pose.orientation
		
		pose_stamped_to_transform = geometry_msgs.msg.PoseStamped()
		pose_stamped_to_transform.header.frame_id = "head_rgbd_sensor_rgb_frame"
		pose_stamped_to_transform.pose.orientation = grasp_pose.orientation    
		pose_stamped_to_transform.pose.position = grasp_pose.position

		# arm.set_joint_value_target([0.5, 0, 0, 0, 0, 0])
		# arm.go()

		current_pose = get_relative_coordinate("map", "hand_palm_link")

		tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		tf2_listener = tf2_ros.TransformListener(tf2_buffer)

		transform = tf2_buffer.lookup_transform("odom",
									   pose_stamped_to_transform.header.frame_id, #source frame
									   rospy.Time(0), #get the tf at first available time
									   rospy.Duration(2.0)) #wait for 1 second

		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped_to_transform, transform)
		print "New coordinates in the map frame 1: "
		print(pose_transformed)

		
		# if self.listener.frameExists("/head_rgbd_sensor_rgb_frame") and self.listener.frameExists("/map"):
		# 	t = self.listener.getLatestCommonTime("/head_rgbd_sensor_rgb_frame", "/map")
		# 	p1 = geometry_msgs.msg.PoseStamped()
		# 	p1.header.frame_id = "head_rgbd_sensor_rgb_frame"
		# 	p1.pose.orientation = grasp_pose.orientation    
		# 	p1.pose.position = grasp_pose.position
		# 	p_in_end_effector = PoseStamped()
		# 	p_in_end_effector = self.listener.transformPose("/map", p1)
		# 	print "Position of the gripper in the map frame:"
		# 	print p_in_end_effector
		# else:
		# 	print "no transform available"
		
		#r = R.from_euler('xyz', [approach, binormal, hand_axis])
		# r = R.from_rotvec([approach, binormal, hand_axis])

		# quat = r.as_quat()
		# print(r.as_euler('xyz', degrees=True))
		# roll, pitch, yaw = r.as_euler('xyz', degrees=True)

		#p_in_end_effector.pose.orientation = orientation

		#p_in_end_effector.pose.position = grasp_pose.position
		current_pose = get_relative_coordinate("map", "hand_palm_link")

		move_arm_neutral()
		move_hand(1)

		try:
			pose_to_perform = geometry_msgs.msg.PoseStamped()
			pose_to_perform.position = current_pose.position
			pose_to_perform.header = current_pose.header
			pose_to_perform.header.stamp = rospy.Time.now()
			pose_to_perform.orientation = pose_transformed.orientation
			self.move_to(pose_transformed, 'whole_body_light')
		except:
			rospy.logerr('fail to grasp')
			return False

		rospy.sleep(1)
		current_pose = get_relative_coordinate("map", "hand_palm_link")

		try:
			pose_to_perform = geometry_msgs.msg.PoseStamped()
			pose_to_perform.position = pose_transformed.position
			pose_to_perform.header = current_pose.header
			pose_to_perform.header.stamp = rospy.Time.now()

			pose_to_perform.orientation = grasp_pose.orientation
			self.move_to(pose_to_perform, 'whole_body_light')
		except:
			rospy.logerr('fail to grasp')
			return False

		move_hand(0)

		return True

	def transform_Pose(self, pose_array):
		pose = geometry_msgs.msg.Pose()
		pose.position.x = pose_array.x
		pose.position.y = pose_array.y
		pose.position.z = pose_array.z
		return pose

	def rotate(self, pose, target):
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.pose.orientation = quaternion_from_euler(pose[0], pose[1], pose[2])
		group = moveit_commander.MoveGroupCommander(target)
		group.allow_replanning(True)
		group.set_pose_target(pose_goal)
		plan = group.go(wait=True)
		# Calling `stop()` ensures that there is no residual movement
		group.stop()
		# It is always good to clear your targets after planning with poses.
		# Note: there is no equivalent function for clear_joint_value_targets()
		group.clear_pose_targets()

	def move_arm(self, pose):
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.pose.orientation = quaternion_from_euler(pose[0], pose[1], pose[2])

	def attached_object(self):
		grasping_group = 'hand'
		touch_links = self.robot.get_link_names(group=grasping_group)
		self.scene.attach_box(eef_link, box_name, touch_links=touch_links)

	
	def create_cloud_indexed(self, object_pose, pointcloud_sub):
		points = []
		cam_source = []

		pts = list(pc2.read_points(pointcloud_sub, field_names=("x", "y", "z", "rgb"), skip_nans=True))
		for pt in pts:
			cam_source.append(Int64(0))

		for i in range(object_pose.x_min.data, object_pose.x_max.data):
			for j in range(object_pose.y_min.data, object_pose.y_max.data):
				points.append(Int64(i + (j*640)))

		self.listener.waitForTransform('/map', '/head_rgbd_sensor_rgb_frame', rospy.Time(), rospy.Duration(2.0))
		(trans,rot) = self.listener.lookupTransform('/map', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))

		point_msg = Point()
		point_msg.x = trans[0]
		point_msg.y = trans[1]
		point_msg.z = trans[2]

		cloud_source_msg = CloudSources()
		cloud_source_msg.cloud = pointcloud_sub
		cloud_source_msg.camera_source = cam_source
		cloud_source_msg.view_points = [point_msg]

		cloud_index_msg = CloudIndexed()
		cloud_index_msg.cloud_sources = cloud_source_msg
		cloud_index_msg.indices = points

		return cloud_index_msg

	def estimate_pose(self, points, pointcloud_sub):
		return list(pc2.read_points(pointcloud_sub, field_names=("x", "y", "z"), skip_nans=True, uvs=points))

if __name__ == "__main__":
	
	grasping_node = Grasping()
	# Here you can switch between two mode: 
	# 1. Continuous detection by ROS subscriber/callback (asynchronous)
	# 2. Synchronous detection via ROS Service (Server/Client-like)

	#grasping_node.continuous_node()
	grasping_node.service_node()



# Ros_goal cmd;
#     int nb_pos = mat_cmd.n_cols;
#     cmd.trajectory.points.resize(nb_pos);
#     cmd.trajectory.joint_names = jointnames;
#     for(int pos=0; pos<nb_pos; pos++)
#     {
#         cmd.trajectory.points[pos].time_from_start = ros::Duration(mat_cmd(0,pos)/2.5+1.0);// /1.5 augmente la vitesse
#         for(int j=0; j<rob.nb_joints(); j++)
#         {
#             cmd.trajectory.points[pos].positions.emplace_back((float)mat_cmd(j+1,pos));
#         }
#     }
#     return std::move(cmd);

# rostopic pub /hsrb/arm_trajectory_controller/follow_joint_trajectory/goal control_msgs/FollowJointTrajectoryActionGoal "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# goal_id:
#   stamp:
#     secs: 0
#     nsecs: 0
#   id: ''
# goal:
#   trajectory:
#     header:
#       seq: 0
#       stamp:
#         secs: 0
#         nsecs: 0
#       frame_id: ''
#     joint_names:
#     - '"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"'
#   goal_time_tolerance: {secs: 0, nsecs: 0}"  acceleration: 0.0}