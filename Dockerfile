FROM ros:melodic-perception

SHELL [ "/bin/bash", "-c" ]

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update && \
    apt-get install -y git ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-ros-numpy ros-$ROS_DISTRO-geometry && \
    apt-get clean

# install bio_ik
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /bio_ik_ws/src && \
    cd /bio_ik_ws/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/TAMS-Group/bio_ik.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /bio_ik_ws

# create workspace folder
#RUN mkdir -p /workspace/src
RUN mkdir -p /workspace

# copy our algorithm to workspace folder
#ADD . /workspace/src
ADD . /workspace

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# Install dependencies
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /workspace/src && \
    sh install.sh


# Run Cmd algorithm
CMD cd /workspace && \
    source devel/setup.bash && \
    roslaunch robobreizh challenge.launch