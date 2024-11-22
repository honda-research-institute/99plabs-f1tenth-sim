# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
ARG TARGETOS TARGETARCH
ARG ROS_DISTRO=humble
ARG PROJECT_WS=sim_ws

FROM --platform=$BUILDPLATFORM ros:${ROS_DISTRO}
ENV DISPLAY=novnc:0.0
SHELL ["/bin/bash", "-c"]
WORKDIR '/sim_ws'

# dependencies
RUN apt update -qq --no-install-recommends; DEBIAN_FRONTEND='noninteractive' apt install -y --no-install-recommends \
      git vim tmux python3-pip libeigen3-dev \
      ros-${ROS_DISTRO}-realsense2-description \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-nav2-amcl \
      ros-${ROS_DISTRO}-nav2-behavior-tree \
      ros-${ROS_DISTRO}-nav2-behaviors \
      ros-${ROS_DISTRO}-nav2-bt-navigator \
      ros-${ROS_DISTRO}-nav2-collision-monitor \
      ros-${ROS_DISTRO}-nav2-common \
      ros-${ROS_DISTRO}-nav2-constrained-smoother \
      ros-${ROS_DISTRO}-nav2-controller \
      ros-${ROS_DISTRO}-nav2-core \
      ros-${ROS_DISTRO}-nav2-costmap-2d ros-${ROS_DISTRO}-costmap-queue \
      ros-${ROS_DISTRO}-nav2-lifecycle-manager \
      ros-${ROS_DISTRO}-nav2-map-server \
      ros-${ROS_DISTRO}-nav2-msgs ros-${ROS_DISTRO}-ackermann-msgs \
      ros-${ROS_DISTRO}-nav2-planner \
      ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
      ros-${ROS_DISTRO}-nav2-smac-planner \
      ros-${ROS_DISTRO}-nav2-smoother \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-nav2-velocity-smoother \
      ros-${ROS_DISTRO}-nav2-voxel-grid \
      ros-${ROS_DISTRO}-nav2-waypoint-follower \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-nav2-rviz-plugins \
      ros-${ROS_DISTRO}-rqt-graph; \
    apt autoclean; apt autoremove

RUN pip3 install transforms3d setuptools==58.2.0 && \
    git clone https://github.com/f1tenth/f1tenth_gym && \
    cd f1tenth_gym && \
    pip3 install -e .

ENTRYPOINT ["/bin/bash"]
