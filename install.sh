#!/bin/bash

set -e  # Exit on error

echo "🔄 Updating system..."
sudo apt update && sudo apt upgrade -y

echo "🐍 Installing Python packages..."
sudo apt install -y python3-pip python3-opencv python3-pytest python3-setuptools

pip3 install --upgrade numpy

echo "🤖 Installing ROS 2 Jazzy + MoveIt + Control + Gazebo..."

sudo apt install -y \
ros-jazzy-moveit-configs-utils \
ros-jazzy-moveit-kinematics \
ros-jazzy-moveit-msgs \
ros-jazzy-moveit-planners \
ros-jazzy-moveit-ros-move-group \
ros-jazzy-moveit-ros-visualization \
ros-jazzy-moveit-ros-warehouse \
ros-jazzy-moveit-setup-assistant \
ros-jazzy-moveit-simple-controller-manager \
ros-jazzy-warehouse-ros-mongo \
\
ros-jazzy-controller-manager \
ros-jazzy-joint-state-broadcaster \
ros-jazzy-joint-trajectory-controller \
ros-jazzy-gz-ros2-control \
\
ros-jazzy-ros-gz-bridge \
ros-jazzy-ros-gz-sim \
\
ros-jazzy-joint-state-publisher \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-robot-state-publisher \
ros-jazzy-rviz2 \
ros-jazzy-rviz-common \
ros-jazzy-rviz-default-plugins \
\
ros-jazzy-action-msgs \
ros-jazzy-control-msgs \
ros-jazzy-geometry-msgs \
ros-jazzy-nav-msgs \
ros-jazzy-rosgraph-msgs \
ros-jazzy-sensor-msgs \
ros-jazzy-shape-msgs \
ros-jazzy-std-msgs \
ros-jazzy-std-srvs \
ros-jazzy-trajectory-msgs \
ros-jazzy-tf2-msgs \
ros-jazzy-tf2-ros \
ros-jazzy-xacro \
ros-jazzy-rosidl-default-generators \
ros-jazzy-rosidl-default-runtime

echo "🌍 Installing Gazebo development libraries..."

sudo apt install -y \
libgz-sim8-dev \
libgz-plugin2-dev \
libgz-common5-dev \
libgz-transport13-dev \
libgz-msgs10-dev

echo "✅ Installation complete!"

echo "🔁 Sourcing ROS 2..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "🚀 Done! Your ROS 2 Jazzy environment is ready."
