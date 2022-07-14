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

#ifndef MARS_TF2_CONVERTER_H
#define MARS_TF2_CONVERTER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mars_ros/VisionSensorState.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mars_tf2
{
///
/// \brief The Converter class is a base interface for any tf2 converters
///
class Converter
{
protected:
  ros::NodeHandle nh_;                    ///< ROS nodehandle
  tf2_ros::TransformBroadcaster br_tf2_;  ///< ROS TF2 broadcaster (publisher)
  ros::Subscriber sub_msg_;               ///< main ROS subscriber

  std::string child_id_{ "child" };    ///< 'to' frame id = child frame of transform, e.g., p_wi -> child_id = 'imu'
  std::string parent_id_{ "parent" };  ///< 'from' frame id = parent frame of transform, e.g., p_wi -> parent_id =
                                       ///< 'parent'; if this parameter is not set, the frame_id from incoming message
                                       ///< will be used

  bool b_override_parent_{ false };  ///< flag to determine if the parent id from message or parameter should be used

  double t_newest_meas_{ 0.0 };  ///< variable to keep track of newest message received

public:
  ///
  /// \brief Converter standard constor of the Converter class. Sets the nodehandle and reads parameters for the frame
  /// ids.
  /// \param nh ROS nodehandle
  ///
  Converter(ros::NodeHandle& nh) : nh_(nh)
  {
    nh_.param<std::string>("child_id", child_id_, child_id_);
    if (nh_.getParam("parent_id", parent_id_))
      b_override_parent_ = true;

    ROS_DEBUG_STREAM("Created Converter with:\n"
                     << "\tchild_id:          " << child_id_ << "\n"
                     << "\tparent_id:         " << parent_id_ << "\n"
                     << "\toverride_parent:   " << (b_override_parent_ ? "true" : "false") << "\n");
  }
};

///
/// \brief The PoseConverter class for converting PoseStamped messages to TF2.
///
class PoseConverter : public Converter
{
private:
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
  PoseConverter(ros::NodeHandle& nh);
};

///
/// \brief The VisionSensorStateConverter class for converting VisionSensorState messages to TF2.
///
class VisionSensorStateConverter : public Converter
{
private:
  tf2_ros::TransformBroadcaster br_tf2_vw_;
  void StateCallback(const mars_ros::VisionSensorState::ConstPtr& msg);

public:
  VisionSensorStateConverter(ros::NodeHandle& nh);
};

}  // namespace mars_tf2

#endif  // MARS_TF2_CONVERTER_H
