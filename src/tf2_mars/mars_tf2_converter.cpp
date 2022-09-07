// Copyright (C) 2022 Martin Scheiber and Christian Brommer, Control of Networked Systems,
// University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>
// and <christian.brommer@ieee.org>.

#include "tf2_mars/mars_tf2_converter.h"

namespace mars_tf2
{
PoseConverter::PoseConverter(ros::NodeHandle& nh) : Converter(nh)
{
  sub_msg_ = nh_.subscribe("pose_in", 1, &PoseConverter::PoseCallback, this);

  ROS_DEBUG_STREAM("Created PoseConverter");
}

void PoseConverter::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("* PoseConverter::PoseCallback");
  // only publish in-order poses to avoid this issue:
  // https://github.com/ros/geometry2/issues/467
  if (msg->header.stamp.toSec() > t_newest_meas_)
  {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = b_override_parent_ ? parent_id_ : msg->header.frame_id;
    transformStamped.child_frame_id = child_id_;
    transformStamped.transform.translation.x = msg->pose.position.x;
    transformStamped.transform.translation.y = msg->pose.position.y;
    transformStamped.transform.translation.z = msg->pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.orientation.w;

    br_tf2_.sendTransform(transformStamped);
    t_newest_meas_ = msg->header.stamp.toSec();
  }
}

PoseCovConverter::PoseCovConverter(ros::NodeHandle& nh) : Converter(nh)
{
  sub_msg_ = nh_.subscribe("pose_in", 1, &PoseCovConverter::PoseCovCallback, this);

  ROS_DEBUG_STREAM("Created PoseConverter");
}

void PoseCovConverter::PoseCovCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("* PoseCovConverter::PoseCovCallback");
  // only publish in-order poses to avoid this issue:
  // https://github.com/ros/geometry2/issues/467
  if (msg->header.stamp.toSec() > t_newest_meas_)
  {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = b_override_parent_ ? parent_id_ : msg->header.frame_id;
    transformStamped.child_frame_id = child_id_;
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

    br_tf2_.sendTransform(transformStamped);
    t_newest_meas_ = msg->header.stamp.toSec();
  }
}

VisionSensorStateConverter::VisionSensorStateConverter(ros::NodeHandle& nh) : Converter(nh)
{
  sub_msg_ = nh_.subscribe("state_in", 1, &VisionSensorStateConverter::StateCallback, this);

  ROS_DEBUG_STREAM("Created VisionSensorStateConverter");
}

void VisionSensorStateConverter::StateCallback(const mars_ros::VisionSensorState::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("* VisionSensorStateConverter::StateCallback");
  // only publish in-order poses to avoid this issue:
  // https://github.com/ros/geometry2/issues/467
  if (msg->header.stamp.toSec() > t_newest_meas_)
  {
    geometry_msgs::TransformStamped transformStamped;

    // publish the ic based on information
    transformStamped.header.stamp = msg->header.stamp;
    transformStamped.header.frame_id = b_override_parent_ ? parent_id_ : msg->header.frame_id;
    transformStamped.child_frame_id = child_id_;
    transformStamped.transform.translation.x = msg->p_ic.x;
    transformStamped.transform.translation.y = msg->p_ic.y;
    transformStamped.transform.translation.z = msg->p_ic.z;
    transformStamped.transform.rotation.x = msg->q_ic.x;
    transformStamped.transform.rotation.y = msg->q_ic.y;
    transformStamped.transform.rotation.z = msg->q_ic.z;
    transformStamped.transform.rotation.w = msg->q_ic.w;
    br_tf2_.sendTransform(transformStamped);

    // publish the vw constant frame ids
    transformStamped.header.frame_id = "vision";
    transformStamped.child_frame_id = "world";
    transformStamped.transform.translation.x = msg->p_vw.x;
    transformStamped.transform.translation.y = msg->p_vw.y;
    transformStamped.transform.translation.z = msg->p_vw.z;
    transformStamped.transform.rotation.x = msg->q_vw.x;
    transformStamped.transform.rotation.y = msg->q_vw.y;
    transformStamped.transform.rotation.z = msg->q_vw.z;
    transformStamped.transform.rotation.w = msg->q_vw.w;
    br_tf2_vw_.sendTransform(transformStamped);

    t_newest_meas_ = msg->header.stamp.toSec();
  }
}

static void PrintHelp()
{
  std::string str = "";
  str += "  Usage: rosrun mars_ros mars_tf2_converter -t [TYPE]\n\n";
  str += "                                   -t [TYPE]  type of converter to use\n";
  str += "                                              currently supported: 'pose', 'posecov', or 'vision'\n\n";
  str += "                                   -h print this help\n\n";
  ROS_WARN_STREAM(str);
}
}  // namespace mars_tf2

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mars_tf2_converter");
  ros::NodeHandle nh("~");

  // setup logging
#ifdef NDEBUG
  // nondebug
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_DEBUG_STREAM("autoset logger level to: INFO");
#else
  // debug
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_DEBUG_STREAM("autoset logger level to: DEBUG");
#endif

  // check arguments
  if (argc < 2)
  {
    ROS_ERROR_STREAM("Wrong number of arguments provided.");
    mars_tf2::PrintHelp();
    return EXIT_FAILURE;
  }

  // create Converter
  mars_tf2::Converter* conv;

  // check first arg
  if (!std::strcmp(argv[1], "-h"))
  {
    mars_tf2::PrintHelp();
    return EXIT_SUCCESS;
  }
  else if (!std::strcmp(argv[1], "-t"))
  {
    // check arguments
    if (argc < 3)
    {
      ROS_ERROR_STREAM("Wrong number of arguments provided.");
      mars_tf2::PrintHelp();
      return EXIT_FAILURE;
    }

    // create converter based on type
    if (!std::strcmp(argv[2], "pose"))
    {
      ROS_DEBUG_STREAM("creating PoseConverter");
      conv = new mars_tf2::PoseConverter(nh);
    }
    else if (!std::strcmp(argv[2], "posecov"))
    {
      ROS_DEBUG_STREAM("creating PoseCovConverter");
      conv = new mars_tf2::PoseCovConverter(nh);
    }
    else if (!std::strcmp(argv[2], "vision"))
    {
      ROS_DEBUG_STREAM("creating VisionSensorStateConverter");
      conv = new mars_tf2::VisionSensorStateConverter(nh);
    }
    else
    {
      ROS_ERROR_STREAM("unknown type '" << std::string(argv[2]) << "'");
      mars_tf2::PrintHelp();
      return EXIT_FAILURE;
    }

    // spin node
    ros::spin();
  }
  else
  {
    ROS_ERROR_STREAM("unknown argument '" << std::string(argv[1]) << "'");
    mars_tf2::PrintHelp();
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
};
