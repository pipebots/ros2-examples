#!/bin/bash
# Adds the example packages to the workspace.
# This is designed to be run from inside the docker.
set -e

# Link the code directories containing the packages to be built.
cd ~/ws/src
CODE_DIR=${HOME}/code
ln -sf ${CODE_DIR}/example_combined
ln -sf ${CODE_DIR}/example_launch
ln -sf ${CODE_DIR}/example_msgs
ln -sf ${CODE_DIR}/example_singles

# Build the packages.
cd ~/ws
. /opt/ros/foxy/setup.bash
colcon build

echo
echo "$0 setup took $SECONDS seconds."
echo
