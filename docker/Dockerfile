# Docker setup for Intent-MPC (ROS Noetic)
# Based on FASTER docker pattern, adapted for Intent-MPC requirements

FROM ros:noetic

# Build argument for cache-busting when using make build-clone
ARG DUMMY=unknown

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Install system utilities
RUN apt-get update && apt-get install -y \
    git \
    wget \
    tmux \
    vim \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install tmuxp
RUN pip3 install tmuxp

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-rviz-plugins \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-mavros-extras \
    ros-noetic-vision-msgs \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-message-filters \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Install C++ libraries
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install numpy pyyaml

# Create workspace directory
RUN mkdir -p /root/ip-mpc_ws/src

# Clone quadrotor_msgs dependency
RUN cd /root/ip-mpc_ws/src && \
    git clone https://github.com/jchenbr/quadrotor_msgs.git

# Copy workspace from host
COPY Intent-MPC /root/ip-mpc_ws/src/Intent-MPC
# COPY acl-gazebo /root/ip-mpc_ws/src/acl-gazebo
# COPY acl-mapping /root/ip-mpc_ws/src/acl-mapping

# Eigen symlink fix
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Python3 symlink
RUN ln -sf /usr/bin/python3 /usr/local/bin/python

# Build workspace with catkin_make
RUN cd /root/ip-mpc_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Setup ROS environment in .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/ip-mpc_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "source /root/ip-mpc_ws/src/Intent-MPC/uav_simulator/gazeboSetup.bash" >> /root/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc && \
    echo "export ROS_IP=127.0.0.1" >> /root/.bashrc && \
    echo "alias tks='tmux kill-session'" >> /root/.bashrc

# Set Gazebo environment variables
ENV GAZEBO_MODEL_PATH=/root/ip-mpc_ws/src/Intent-MPC/uav_simulator/models:/root/ip-mpc_ws/src/acl-gazebo/acl_sim/models
ENV GAZEBO_RESOURCE_PATH=/root/ip-mpc_ws/src/Intent-MPC/uav_simulator:/root/ip-mpc_ws/src/acl-gazebo/acl_sim

# Set working directory
WORKDIR /root/ip-mpc_ws

# Default command
CMD ["/bin/bash"]
