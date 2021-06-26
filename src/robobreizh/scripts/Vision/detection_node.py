#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import os

os.environ["DARKNET_PATH"] = "/workspace/src/dependencies/darknet/"

import rospy

from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointCloud, PointField
from geometry_msgs.msg import Point32, Point, PoseStamped, Pose, Twist

import numpy as np
import time
from threading import Thread, Event

import cv2
from cv_bridge import CvBridge, CvBridgeError

import darknet
import json
import random

from std_msgs.msg import String, Header, Int64
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import tf
import message_filters
#import pcl
from std_msgs.msg import Float32, Int64
from robobreizh.msg import CloudIndexed, CloudSources, GraspConfigList, DetectedObj, GraspConfig, BoundingBoxCoord
from robobreizh.srv import detect_grasps, object_detection


class ObjectsDetection():
	def __init__(self):
		rospy.init_node('ObjectsDetectionNode', anonymous=False)

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

		self.bridge = CvBridge()
		self.listener = tf.TransformListener()
		self.object_pose = [0,0,0,0]
		self.cloud_points = []

		# PUBLISHERS
		self.detect_pub = rospy.Publisher('/detected_object', String, queue_size=10)
		self.visualize_detect_pub = rospy.Publisher('/visualize_pointcloud', PointCloud2, queue_size=10)
		self.object_pc_pub = rospy.Publisher("crop_pointcloud", PointCloud2, queue_size=10)
		self.cloud_index_pub = rospy.Publisher("gpd_cloud_indexed", CloudIndexed, queue_size=10)
		# The following ones are unusued for now
		self.cropped_depth_image_pub = rospy.Publisher("cropped_depth_image", Image, queue_size=10)
		self.cropped_rgb_image_pub = rospy.Publisher("cropped_rgb_image", Image, queue_size=10)
		self.cropped_camera_info = rospy.Publisher("cropped_camera_info", CameraInfo, queue_size=10)

		# SUBSCRIBERS
		self.image_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image)
		self.pointcloud_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
		self.depth_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image)


	def continuous_node(self):

		# publisher_thread = Thread(target=self.object_thread)
		# publisher_thread.start()

		ts = message_filters.TimeSynchronizer([self.image_sub, self.pointcloud_sub, self.depth_sub], 10)
		ts.registerCallback(self.callback)

		rospy.loginfo("Launching Detection Node")

		rospy.spin()
		#publisher_thread.join()

	def service_node(self):
		s = rospy.Service('object_detection', object_detection, self.handle_object_detection)
		rospy.loginfo("Object Detection Service Node: Waiting for Request...")
		rospy.spin()

	def handle_object_detection(self, req):
		return self.main_loop()

	def detect_dummy_object():
		image_data = rgbd.get_image()
		points_data = rgbd.get_points()
		h_image = rgbd.get_h_image()
		rgbd.set_h(130, 140)
		region = rgbd.get_region()

		interact(f, lower=(0, 255, 5), upper=(0, 255, 5))

	def main_loop(self):
		
		image_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/image_rect_color', Image)
		pointcloud_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
		depth_sub = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw', Image)

		image_name = self.bridge.imgmsg_to_cv2(image_sub)

		image, detections = self.image_detection(image_name, self.network, self.class_names, self.class_colors)
		darknet.print_detections(detections, False)

		# Uncomment to visualize detection in real time (ressource consuming)
		# cv2.imshow('Inference', image)
		# cv2.waitKey(1)
	
		detected_obj = {}
		points = []
		point_clouds = []
		labels = []
		bounding_boxes = []
		threshold = 40.0
		
		for obj in detections:
			if (float(obj[1]) > threshold): #Check if the confidence is above a threshold
				labels.append(String(obj[0]))
				x, y, w, h = obj[2][0], obj[2][1], obj[2][2], obj[2][3]

				boundingbox = BoundingBoxCoord()
				x_min, y_min, x_max, y_max = self.convertBack(x,y,w,h)
				boundingbox.x_min, boundingbox.y_min, boundingbox.x_max, boundingbox.y_max = (Int64(x) for x in self.convertBack(x,y,w,h))

				bounding_boxes.append(boundingbox)

				points.append([int(x), int(y)])
				#pc = self.crop_object_pointcloud(x_min, y_min, x_max, y_max, [x,y], pointcloud_sub)
				#point_clouds.append(pc)

		if not labels:
			final_msg = DetectedObj()
			final_msg.object_names = [String("nothing")]
			return final_msg
		
		poses = self.estimate_pose(points, pointcloud_sub)
		print(points)
		obj_poseXYZ = []
		for pos in poses:
			temp = Pose()
			temp.position.x = pos[0]
			temp.position.y = pos[1]
			temp.position.z = pos[2]

			obj_poseXYZ.append(temp)

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
		  PointField('y', 4, PointField.FLOAT32, 1),
		  PointField('z', 8, PointField.FLOAT32, 1),
		  ]
		header = Header()
		header.stamp = rospy.Time.now()
		poses_msg = pc2.create_cloud(header, fields, poses)
		self.visualize_detect_pub.publish(poses_msg)

		final_msg = DetectedObj()
		final_msg.object_names = labels
		final_msg.objects_bb = bounding_boxes
		final_msg.object_poses = poses_msg
		final_msg.cloud = pointcloud_sub
		final_msg.object_posesXYZ = obj_poseXYZ
		print(obj_poseXYZ)
		return final_msg


	def estimate_pose(self, points, pointcloud_sub):
		res = []
		gen = pc2.read_points(pointcloud_sub, field_names=("x", "y", "z"), skip_nans=True, uvs=points)
		for p in gen:
			res.append(p)
		return res


	########################################################
	###                     UNUSUED						 ###
	########################################################

	def crop_object_pointcloud(self, min_x, min_y, max_x, max_y, center_coord, pointcloud_sub):
		points = []
		for i in range(min_x, max_x):
			for j in range(min_y, max_y):
				points.append([i, j])

		center_coord = [int(round(center_coord[0])), int(round(center_coord[1]))]
		points = list(pc2.read_points(pointcloud_sub, field_names=("x", "y", "z", "rgb"), skip_nans=True, uvs=points))
		center = list(pc2.read_points(pointcloud_sub, field_names=("x", "y", "z"), skip_nans=True, uvs=[center_coord, [0,0]]))[0]

		pcl_data = pcl.PointCloud_PointXYZRGB()
		pcl_data.from_list(points)

		objects_cloud = self.use_ransac(pcl_data) 

		colorless_cloud = XYZRGB_to_XYZ(objects_cloud)
		clusters = self.clustering(colorless_cloud)

		# # Get groups of indices for each cluster of points
		# # Each group of points belongs to the same object
		# # This is effectively a list of lists, with each list containing indices of the cloud
		# #clusters = self.get_clusters(colorless_cloud, tolerance = 0.05, min_size = 100, max_size = 1000)

		final_points = []
		# r = randint(0, 255)
  #   	g = randint(0, 255)
  #   	b = randint(0, 255)

		# No clustering if there is only one cluster for speedup
		if len(clusters) == 1:
			final_points = np.array(objects_cloud)
		else:

			# We get the points belonging to each clusters
			# Two methods: 
			# 1. Get the cluster where the center point of the bounding box belong
			# 2. Get the biggest size cluster (most likely the object)

			method = "2"

			# Method 1
			if method == "1":
				pt_index = ""
				center = [round(x) for x in center]
				colorless_cloud_round = []

				for pt in colorless_cloud:
					pt_round = [round(x) for x in pt]
					colorless_cloud_round.append(pt_round)

				for i in range(len(colorless_cloud_round)):
					if center == colorless_cloud_round[i]:
						pt_index = i

				for cluster in clusters:
					if pt_index in cluster:
						for c, i in enumerate(cluster):
							x, y, z, rgb = objects_cloud[i][0], objects_cloud[i][1], objects_cloud[i][2], objects_cloud[i][3]
							final_points.append([x, y, z, rgb])
						break

			# Method 2
			else:
				max_size = 0
				max_cluster = []
				for cluster in clusters:
					if len(cluster) > max_size:
						max_cluster = cluster
						max_size = len(cluster)
						
				for c, i in enumerate(max_cluster):
					x, y, z, rgb = objects_cloud[i][0], objects_cloud[i][1], objects_cloud[i][2], objects_cloud[i][3]
					final_points.append([x, y, z, rgb])


		# fields = [PointField('x', 0, PointField.FLOAT32, 1),
		# 	PointField('y', 4, PointField.FLOAT32, 1),
		# 	PointField('z', 8, PointField.FLOAT32, 1),
		# 	PointField('rgba', 16, PointField.FLOAT32, 1),

		# ]

		# header = pointcloud_sub.header
		# pc_msg = pc2.create_cloud(header, fields, final_points)

		# pc_msg.header.stamp = rospy.Time.now()
		# return pc_msg
		#self.object_pc_pub.publish(pc_msg)
	 	clusters_cloud = pcl.PointCloud_PointXYZRGB()
  		clusters_cloud.from_list(final_points)
		objects_msg = pcl_to_ros(clusters_cloud)
		return objects_msg


	def use_ransac(self, points):
		fil = points.make_passthrough_filter()
		fil.set_filter_field_name("z")
		fil.set_filter_limits(0, 1.5)
		cloud_filtered = fil.filter()

		seg = cloud_filtered.make_segmenter_normals(ksearch=100)
		seg.set_optimize_coefficients(True)
		seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
		seg.set_normal_distance_weight(0.1)
		seg.set_method_type(pcl.SAC_RANSAC)
		seg.set_max_iterations(100)
		seg.set_distance_threshold(0.01)
		indices, model = seg.segment()

		object_cloud = cloud_filtered.extract(indices, negative=True)
		#table_cloud = cloud_filtered.extract(indices, negative=False)

		return object_cloud

	def object_thread(self):
		rate = rospy.Rate(100) # ROS Rate at 100Hz

		while not rospy.is_shutdown():
			# fields = [PointField('x', 0, PointField.FLOAT32, 1),
			# 	PointField('y', 4, PointField.FLOAT32, 1),
			# 	PointField('z', 8, PointField.FLOAT32, 1),
			# 	PointField('rgb', 16, PointField.FLOAT32, 1),
			# ]
			
			# header = Header()
			# header.stamp = rospy.Time.now()
			# header.frame_id = "head_rgbd_sensor_rgb_frame"
			# pc_msg = pc2.create_cloud(header, fields, self.object_pose)

			# self.object_pc_pub.publish(pc_msg)

			min_x, min_y, max_x, max_y = self.object_pose

			points = []
			for i in range(min_x, max_x):
				for j in range(min_y, max_y):
					points.append(int(i + (j*640)))

			self.listener.waitForTransform('/map', '/head_rgbd_sensor_rgb_frame', rospy.Time(), rospy.Duration(2.0))
			(trans,rot) = self.listener.lookupTransform('/map', '/head_rgbd_sensor_rgb_frame', rospy.Time(0))

			point_msg = Point()
			point_msg.x = trans[0]
			point_msg.y = trans[1]
			point_msg.z = trans[2]

			cloud_source_msg = CloudSources()
			cloud = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
			cloud_source_msg.cloud = cloud
			cloud_source_msg.camera_source = [int(1)]
			cloud_source_msg.view_points = [point_msg]

			cloud_index_msg = CloudIndexed()
			cloud_index_msg.cloud_sources = cloud_source_msg
			cloud_index_msg.indices = points

			self.cloud_index_pub.publish(cloud_index_msg)
			
			rate.sleep()

	def callback(self, image_sub, pointcloud_sub, depth_sub):

		image_name = self.bridge.imgmsg_to_cv2(image_sub)

		image, detections = self.image_detection(image_name, self.network, self.class_names, self.class_colors)
		darknet.print_detections(detections, False)

		#cv2.imshow('Inference', image)
		#cv2.waitKey(1)
	
		detected_obj = {}
		points = []

		if detections:
			x, y, w, h = detections[1][2][0], detections[1][2][1], detections[1][2][2], detections[1][2][3]
			x_start, y_start, x_end, y_end = self.convertBack(x,y,w,h)
			# depth_im = self.crop_image(x_start, y_start, x_end, y_end, self.bridge.imgmsg_to_cv2(depth_sub, "16UC1"))
			# rgb_img = self.crop_image(x_start, y_start, x_end, y_end, self.bridge.imgmsg_to_cv2(image_sub, "bgr8"))
			# rgb_msg = self.bridge.cv2_to_imgmsg(rgb_img, "bgr8")
			# depth_msg = self.bridge.cv2_to_imgmsg(depth_im, "16UC1")
			#publish camera info topic on the new size

			# camera_info_msg = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo)
			# camera_info_msg.width = w
			# camera_info_msg.height = h
			# camera_info_msg.header = std_msgs.msg.Header()
			# camera_info_msg.header.stamp = rospy.Time.now()

			# self.cropped_depth_image_pub.publish(depth_msg)
			# self.cropped_rgb_image_pub.publish(rgb_msg)
			# self.cropped_camera_info.publish(camera_info_msg)
			#self.create_cloud_indexed(x_start, y_start, x_end, y_end, pointcloud_sub)
			#self.crop_pointcloud(x_start, y_start, x_end, y_end, [x,y], pointcloud_sub)
			# self.object_pose = [x_start, y_start, x_end, y_end]
			# self.cloud_points = pointcloud_sub

			msg = self.create_cloud_indexed([x_start, y_start, x_end, y_end], pointcloud_sub)

			rospy.wait_for_service('/detect_grasps_server/detect_grasps')
   			grasp_service = rospy.ServiceProxy('/detect_grasps_server/detect_grasps', detect_grasps)
   			try:
   		   		resp = grasp_service(msg)
   			except rospy.ServiceException as exc:
   				print("Service did not process request: " + str(exc))
   			print(resp.grasp_configs.grasps[0])

		for obj in detections:
			x, y, w, h = obj[2][0], obj[2][1], obj[2][2], obj[2][3]
			points.append([int(x), int(y)])

		poses = self.estimate_pose(points, pointcloud_sub)
		fields = [PointField('x', 0, PointField.FLOAT32, 1),
		  PointField('y', 4, PointField.FLOAT32, 1),
		  PointField('z', 8, PointField.FLOAT32, 1),
		  ]
		header = pointcloud_sub.header
		pc_msg = pc2.create_cloud(header, fields, poses)

		pc_msg.header.stamp = rospy.Time.now()
		
		self.visualize_detect_pub.publish(pc_msg)

		i = 0
		for obj in detections:
			detected_obj[obj[0]] = poses[i]
			i = i+1
		
		detected_obj_msg = json.dumps(detected_obj)
		self.detect_pub.publish(detected_obj_msg)

		if not detected_obj and time_elapsed >= 10:
			return False
		
		# for obj in detected_obj:
		# 	br = tf.TransformBroadcaster()
		# 	br.sendTransform(detected_obj[obj], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), obj, 'head_rgbd_sensor_rgb_frame')

	def crop_pointcloud(self, min_x, min_y, max_x, max_y, center_coord, pointcloud_sub):
		points = []
		for i in range(min_x, max_x):
			for j in range(min_y, max_y):
				points.append([i, j])

		points = list(pc2.read_points(pointcloud_sub, field_names=("x", "y", "z", "rgb"), skip_nans=True, uvs=points))

		# fields = [PointField('x', 0, PointField.FLOAT32, 1),
		# 	PointField('y', 4, PointField.FLOAT32, 1),
		# 	PointField('z', 8, PointField.FLOAT32, 1),
		# 	PointField('rgb', 16, PointField.FLOAT32, 1),
		# ]

		# header = Header()
		# pc_msg = pc2.create_cloud(header, fields, points)

		# pc_msg.header.stamp = rospy.Time.now()
		# pc_msg.header.frame_id = "head_rgbd_sensor_rgb_frame"
		self.object_pose =  points
		#self.object_pc_pub.publish(pc_msg)

	

	def clustering(self, cloud):
		# vg = cloud.make_voxel_grid_filter()
		# vg.set_leaf_size(0.01, 0.01, 0.01)
		# cloud_filtered = vg.filter()

		# nr_points = cloud_filtered.size
		# print(nr_points)

		# Creating the KdTree object for the search method of the extraction
		tree = cloud.make_kdtree()

		ec = cloud.make_EuclideanClusterExtraction()
		ec.set_ClusterTolerance (0.005)
		ec.set_MinClusterSize (10)
		ec.set_MaxClusterSize (1000)
		ec.set_SearchMethod (tree)
		cluster_indices = ec.Extract()

		#print('cluster_indices : ' + str(cluster_indices[0]))
		return cluster_indices


	# This pipeline separates the objects in the table from the given scene
	def split_cloud(self, cloud):

		# Downsample the cloud as high resolution which comes with a computation cost
		downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.01)

		# Get only information in our region of interest as we don't care about the other parts
		filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
											 name_axis = 'z', min_axis = 0.6, max_axis = 1.1)

		# Separate the table from everything else
		table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 0.01)

		return objects_cloud, table_cloud


	# This pipeline returns groups of indices for each cluster of points
	# Each cluster of indices is grouped as belonging to the same object
	# This uses DBSCAN Algorithm Density-Based Spatial Clustering of Applications with noise
	# Aka Eucledian clustering to group points 
	def get_clusters(self, cloud, tolerance, min_size, max_size):

		tree = cloud.make_kdtree()
		extraction_object = cloud.make_EuclideanClusterExtraction()

		extraction_object.set_ClusterTolerance(tolerance)
		extraction_object.set_MinClusterSize(min_size)
		extraction_object.set_MaxClusterSize(max_size)
		extraction_object.set_SearchMethod(tree)

		# Get clusters of indices for each cluster of points, each cluster belongs to the same object
		# 'clusters' is effectively a list of lists, with each list containing indices of the cloud
		clusters = extraction_object.Extract()
		return clusters


	def image_detection(self, image, network, class_names, class_colors, thresh=0.25):
		# Darknet doesn't accept numpy images.
		# Create one with image we reuse for each detect
		# width = darknet.network_width(network)
		# height = darknet.network_height(network)
		width = 640
		height = 480
		height = darknet.network_height(network)
		darknet_image = darknet.make_image(width, height, 3)

		image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		image_resized = cv2.resize(image_rgb, (width, height),
								   interpolation=cv2.INTER_LINEAR)

		darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
		detections = darknet.detect_image(
			network, class_names, darknet_image, thresh=thresh)
		darknet.free_image(darknet_image)
		image = darknet.draw_boxes(detections, image_resized, class_colors)
		return cv2.cvtColor(image, cv2.COLOR_BGR2RGB), detections

	def convertBack(self, x, y, w, h):
		xmin = int(round(x - (w / 2)))
		xmax = int(round(x + (w / 2)))
		ymin = int(round(y - (h / 2)))
		ymax = int(round(y + (h / 2)))
		return xmin, ymin, xmax, ymax

	def crop_image(self, min_x, min_y, max_x, max_y, image):
		image_cropped = image[min_y:max_y, min_x:max_x]
		return image_cropped

	def get_depth(self, x, y):
		gen = pc2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[(x, y)])
		return next(gen)

	def create_pointcloudXYZRGB(self, rgb_img, depth_im, camera_info):
		self.depth = img.frombytes("F", (depth_im.width, depth_im.height), depth_im.data)
		lookup_depth = self.depth.load()
		points = []

		self.model.fromCameraInfo(camera_info.data)
		for i in range(depth_im.width):
			for j in range(depth_im.heigth):
				depth = lookup_depth[i, j]
				ray = self.model.projectPixelTo3dRay(tuple([i,j]))
				ray_z = [el / ray[2] for el in ray]  # normalize the ray so its Z-component equals 1.0
				pt = [el * depth for el in ray_z]  # multiply the ray by the depth; its Z-component should now equal the depth value
				points.append(pt)

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
				  PointField('y', 4, PointField.FLOAT32, 1),
				  PointField('z', 8, PointField.FLOAT32, 1),
				  ]

		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = "head_rgbd_sensor_rgb_frame"
		self.object_pc_pub = point_cloud2.create_cloud(header, fields, points)
		self.object_pc_pub.header.stamp = rospy.Time.now()
		pub.publish(self.object_pc_pub)

			
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


if __name__ == "__main__":
	
	obj_detection_node = ObjectsDetection()

	# Here you can switch between two mode: 
	# 1. Continuous detection by ROS subscriber/callback (asynchronous)
	# 2. Synchronous detection via ROS Service (Server/Client-like)

	#obj_detection_node.continuous_node()
	obj_detection_node.service_node()