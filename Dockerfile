FROM ros:melodic-ros-base

# install ros tutorials packages
RUN apt-get update && apt-get install -y sudo \
    ros-melodic-ros-tutorials \
    ros-melodic-common-tutorials \
    && rm -rf /var/lib/apt/lists/

# add rosuser so we don't use root
RUN groupadd -g 999 rosuser && \
    useradd -r -u 999 -ms /bin/bash -d /home/rosuser -g rosuser rosuser && \
    usermod -aG sudo rosuser
RUN echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER rosuser
WORKDIR /home/rosuser

# we need to source the setup.bash
SHELL ["/bin/bash", "-c"]

# update ROS packages and create workspace
RUN rosdep update \
    && source /opt/ros/melodic/setup.bash \
    && mkdir -p ~/ros_ws/src \
    && cd ~/ros_ws/ \
    && catkin_make
