#!/bin/bash
# Adds the example packages to the workspace.
# This is designed to be run from inside the docker.
set -e

# Link the code directories containing the packages to be built.
cd ~/ws/src
CODE_DIR=${HOME}/code
ln -sf ${CODE_DIR}/combined
ln -sf ${CODE_DIR}/leeds_pump_launch
ln -sf ${CODE_DIR}/leeds_pump_msgs
ln -sf ${CODE_DIR}/single

# Build the packages.
cd ~/ws
. /opt/ros/foxy/setup.bash
colcon build

echo
echo "$0 setup took $SECONDS seconds."
echo
