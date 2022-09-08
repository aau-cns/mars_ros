#!/bin/bash -e
echo "Copyiing Source Files..."
cp -r /source/ /catkin_build_ws/src/mars_ros
cp -r /source/ /catkin_make_ws/src/mars_ros

# build with catkin build
echo "Building the Appliaction with 'catkin build' ..."
cd /catkin_build_ws
catkin build -DCMAKE_BUILD_TYPE=Release
catkin build -DCMAKE_BUILD_TYPE=Debug
echo "Building the Appliaction with 'catkin build' - DONE"
# clean workspace to free up used disk space
catkin clean -y
rm -rf .catkin_tools

# build with catkin_make
echo "Building the Appliaction with 'catkin_make' ..."
cd /catkin_make_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make -DCMAKE_BUILD_TYPE=Debug
echo "Building the Appliaction with 'catkin_make' - DONE"
# clean workspace to free up used disk space
rm -rf build develop
