#!/bin/bash -e
echo "Copy Source Files..."
cp -r /source/ /catkin_ws/src/mars_ros

# build with catkin build
echo "Building the Appliaction with 'catkin build' ..."
cd /catkin_ws
catkin build -DCMAKE_BUILD_TYPE=Release
catkin build -DCMAKE_BUILD_TYPE=Debug
echo "Building the Appliaction with 'catkin build' - DONE"

# reset workspace and build again with catkin_make for full portability
catkin clean -y
rm -rf .catkin_tools

# build with catkin_make
echo "Building the Appliaction with 'catkin_make' ..."
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make -DCMAKE_BUILD_TYPE=Debug
echo "Building the Appliaction with 'catkin_make' - DONE"
