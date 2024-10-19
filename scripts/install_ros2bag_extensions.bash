#!/bin/bash

WORKSPACE_DIR=~/extension_ws


if [ ! -d "$WORKSPACE_DIR/src" ]; then
  echo "Creating directory $WORKSPACE_DIR/src..."
  mkdir -p $WORKSPACE_DIR/src
fi

echo "Cloning ros2bag_extensions repository..."
cd $WORKSPACE_DIR/src
git clone https://github.com/tier4/ros2bag_extensions.git

echo "Installing dependencies..."
cd $WORKSPACE_DIR
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO

echo "Building the workspace..."
colcon build --symlink-install --catkin-skip-building-tests --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release

source $WORKSPACE_DIR/install/setup.bash

echo "Installation complete!"
