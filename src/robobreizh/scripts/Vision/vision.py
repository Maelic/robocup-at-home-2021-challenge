#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointCloud, PointField
from geometry_msgs.msg import Point32

import numpy as np
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import darknet
import json
import os
import random
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
libdarknet_path = os.environ["DARKNET_PATH"] = "/workspace/src/dependencies/darknet/"
import tf
from utils import *
import message_filters

class Vision(object):
	def __init__(self):
		self.darknet_config_dir = os.path.join(os.environ.get('DARKNET_PATH', './'), "darknet_config/")
		self.config_file = self.darknet_config_dir+"yolov4-tiny-obj.cfg"
		self.data_file = self.darknet_config_dir+"obj.data"
		self.weights = self.darknet_config_dir+"yolov4-tiny-obj_last.weights"
		random.seed(3)  # deterministic bbox colors
		self.network, self.class_names, self.class_colors = darknet.load_network(
			self.config_file,
			self.data_file,
			self.weights,
			batch_size=1
		)

	def image_detection(self, image, thresh=0.25):
		# Darknet doesn't accept numpy images.
		# Create one with image we reuse for each detect
		width = 640
		height = 480
		height = darknet.network_height(self.network)
		darknet_image = darknet.make_image(width, height, 3)

		image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		image_resized = cv2.resize(image_rgb, (width, height),
								   interpolation=cv2.INTER_LINEAR)

		darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
		detections = darknet.detect_image(
			self.network, self.class_names, darknet_image, thresh=thresh)
		darknet.free_image(darknet_image)
		image = darknet.draw_boxes(detections, image_resized, self.class_colors)
		return cv2.cvtColor(image, cv2.COLOR_BGR2RGB), detections

	def searchObject(self):
		detect_pub = rospy.Publisher('/detected_object', String, queue_size=10)
		visualize_detect = rospy.Publisher('/visualize_pointcloud', PointCloud, queue_size=10)
		bridge = CvBridge()
		start_time = time.time()

		while True:
			data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_raw', Image)

			image_name = bridge.imgmsg_to_cv2(data)

			image, detections = self.image_detection(image_name)

			darknet.print_detections(detections, False)

			cv2.imshow('Inference', image)
			cv2.waitKey(1)

			detected_obj = {}
			points = []

			if detected_obj:
				x, y, w, h = detections[0][2][0], detections[0][2][1], detections[0][2][2], detections[0][2][3]
				self.crop_pointcloud(x, y, w, h)

			for obj in detections:
				x, y, w, h = obj[2][0], obj[2][1], obj[2][2], obj[2][3]
				points.append([int(x), int(y)])
			poses = self.estimate_pose(points)

			i = 0
			for obj in detections:
				detected_obj[obj[0]] = poses[i]
				i = i+1

			# print("\nLocations:")
			# for obj in detected_obj:
			# 	print("{}: {}".format(obj, detected_obj[obj]))
			detected_obj_msg = json.dumps(detected_obj)
			detect_pub.publish(detected_obj_msg)

			msg = self.create_pointcloud(poses)
			visualize_detect.publish(msg)

			time_elapsed = time.time() - start_time

			if not detected_obj and time_elapsed >= 10:
				return False

			for obj in detected_obj:
				br = tf.TransformBroadcaster()
				br.sendTransform(detected_obj[obj], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), obj, 'head_rgbd_sensor_rgb_frame')
				print("Relative coord:")
				print(get_relative_coordinate("map", obj))

			if detected_obj:
				return detected_obj
			
	def compute_box_coordinates(self, x1, y1, w_size, h_size):
		x_top_left 		= 	int(round(x1 - (w_size/2)))
		y_top_left 		= 	int(round(y1 - (h_size/2)))
		x_bottom_right  = 	int(round(x_start + w_size))
		y_bottom_right  = 	int(round(y_start + h_size))
		x_top_right 	= 	x_bottom_right
		y_top_right 	= 	y_top_left
		x_bottom_left 	= 	x_top_left
		y_bottom_left 	= 	y_bottom_right

		return [(x_top_left, y_top_left), (x_top_right, y_top_right), (x_bottom_right, y_bottom_right), (x_bottom_left, y_bottom_left)]


	def crop_pointcloud(self, min_x, min_y, max_x, max_y, width, height):

		pc_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
		pub = rospy.Publisher("crop_pointcloud", PointCloud2, queue_size=10)

		points = []
		def perimeter(min_x, min_y, max_x, max_y):
			x, y = min_x, min_y
			for dx, dy in (1, 0), (0, 1), (-1, 0), (0, -1):
				while x in range(min_x, max_x+1) and y in range(min_y, max_y+1):
					yield (x, y)
					x += dx
					y += dy
				x -= dx
				y -= dy

		for p in perimeter(min_x, min_y, max_x, max_y):
			points.append(p)


		fields = [PointField('x', 0, PointField.FLOAT32, 1),
		  PointField('y', 4, PointField.FLOAT32, 1),
		  PointField('z', 8, PointField.FLOAT32, 1),
		  PointField('rgb', 16, PointField.UINT32, 1),
		  ]

		points = list(pc2.read_points(pc_sub, field_names=("x", "y", "z", "rgb"), skip_nans=True, uvs=points))
		header = pc_sub.header
		pc2 = point_cloud2.create_cloud(header, fields, points)

		pc2.header.stamp = rospy.Time.now()
		pub.publish(pc2)

	def create_pointcloud(self, poses):
		pc_msg = PointCloud()
		pc_msg.header = std_msgs.msg.Header()
		pc_msg.header.stamp = rospy.Time.now() 
		pc_msg.header.frame_id = 'head_rgbd_sensor_rgb_frame'

		for pos in poses:
			pc_msg.points.append(Point32(pos[0],pos[1],pos[2]))

		return pc_msg

	def estimate_pose(self, points):
		pc_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
		return(list(pc2.read_points(pc_sub, field_names=("x", "y", "z"), skip_nans=True, uvs=points)))
