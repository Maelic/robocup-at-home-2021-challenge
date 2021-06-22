#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
from utils import *
import cv2
import subprocess
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

rospy.init_node('vision')

def take_picture():
	bridge = CvBridge()
	image_depth = rospy.wait_for_message("/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image) 
	image_rgb = rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_raw", Image) 

	imageRGB = bridge.imgmsg_to_cv2(image_rgb, "bgr8")
	cv_image = bridge.imgmsg_to_cv2(image_depth, "32FC1")
	cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
	imageDepth = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)

	cv2.imwrite('images/img_color.png', imageRGB)
	cv2.imwrite('images/img_depth.png', imageDepth)

def show_objects_pose():
	python3_command = "~/workspace/src/dependencies/PoseCNN-PyTorch/tools/test_images.py --gpu 0 \
  --imgdir ~/workspace/src/robobreizh/scripts/images \
  --meta ~/workspace/src/dependencies/PoseCNN-PyTorch/data/demo/meta.yml \
  --color *color.png \
  --network posecnn \
  --pretrained ~/workspace/src/dependencies/PoseCNN-PyTorch/data/checkpoints/ycb_object/vgg16_ycb_object_self_supervision_epoch_8.checkpoint.pth \
  --dataset ycb_object_test \
  --cfg ~/workspace/src/dependencies/PoseCNN-PyTorch/experiments/cfgs/ycb_object.yml" 

	process = subprocess.Popen(python3_command.split(), stdout=subprocess.PIPE)
	output, error = process.communicate()  

if __name__=='__main__':
    take_picture()
