// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <ros/ros.h>

#ifdef POSE
#include "mars_wrapper_pose.h"
#endif
#ifdef POSITION
#include "mars_wrapper_position.h"
#endif
#ifdef GPS
#include "mars_wrapper_gps.h"
#endif

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mars_ros_node");
  ros::NodeHandle nh("~");

  if (nh.param<bool>("use_tcpnodelay", false))
  {
    ROS_INFO("Using tcpNoDelay ");
    ros::TransportHints().tcpNoDelay();
  }

  ROS_INFO("Starting the MaRS Framework");

#ifdef POSE
  MarsWrapperPose mars_core(nh);
#endif
#ifdef POSITION
  MarsWrapperPosition mars_core(nh);
#endif
#ifdef GPS
  MarsWrapperGps mars_core(nh);
#endif

  ros::spin();
}
