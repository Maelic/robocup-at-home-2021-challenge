catkin_make
source devel/setup.bash
export DARKNET_PATH=/workspace/src/dependencies/darknet/
export FORCE_CPU=true
export CUDA_VISIBLE_DEVICES=-1
export ROS_MASTER_URI=http://172.18.0.3:11311
export ROS_IP=172.18.0.1

rosrun robobreizh detection_node.py

