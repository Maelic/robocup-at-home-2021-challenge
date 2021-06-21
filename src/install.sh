mkdir -p dependencies
BASE=/workspace/src
DEPEND=$BASE/dependencies

sudo apt-get update
sudo apt-get install software-properties-common -y
sudo apt-get install wget curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get install ros-melodic-catkin python-catkin-tools -y
sudo apt-get install --upgrade python-pip -y
pip install scipy

cd $DEPEND 
git clone https://github.com/AlexeyAB/darknet.git

cd $BASE
cp ./Makefile_darknet_CPU ./dependencies/darknet/Makefile 

cd $DEPEND/darknet
#sudo make clean -w &&
sudo make -j4 -w

cd $BASE
cp -r ./darknet_config ./dependencies/darknet/darknet_config 

# INSTALL PCL FROM PPA (NOT WORKING)

# sudo add-apt-repository ppa:bennebo/pcl-1.9.1 -y
# sudo apt-get update
# sudo apt-get install libvtk7-jni -y
# sudo apt-get install libvtk7-dev -y
sudo apt-get install libpcl-dev -y

# INSTALL PCL FROM SOURCES

# cd $DEPEND
# wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.9.1.tar.gz
# tar -xzvf pcl-1.9.1.tar.gz
# cd pcl-pcl-1.9.1
# mkdir -p build && cd build
# cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \ -DBUILD_GPU=ON-DBUILD_apps=ON -DBUILD_examples=ON \ -DCMAKE_INSTALL_PREFIX=/usr ..
# make -j4
# sudo make install

# INSTALL GPD

cd $DEPEND
git clone https://github.com/Maelic/gpd.git
cd gpd
mkdir -p build && cd build
cmake ..
make -j$(($(nproc) - 2))
sudo make install

# PCL CONVERSIONS ROS DEPENDENCIES BUILD FOR PCL 1.9
sudo apt-get install ros-melodic-pcl-msgs -y
sudo apt-get install ros-melodic-pcl-conversions -y

cd $BASE
git clone https://github.com/kunaltyagi/perception_pcl.git
cd ..
catkin_make --only-pkg-with-deps perception_pcl

sudo rm -r build
sudo rm -r devel

catkin build


