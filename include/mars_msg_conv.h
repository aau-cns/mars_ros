// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARS_MSG_CONV_H
#define MARS_MSG_CONV_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mars/core_state.h>
#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_state_type.h>

#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_state_type.h>
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/sensors/position/position_sensor_state_type.h>
#include <mars/type_definitions/core_state_type.h>
#include <mars_ros/ExtCoreState.h>
#include <mars_ros/ExtCoreStateLite.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

///
/// \brief The MarsMsgConv class Conversion between MaRS types and ROS messages
///
class MarsMsgConv
{
public:
  MarsMsgConv() = default;

  static inline mars::IMUMeasurementType ImuMsgToImuMeas(const sensor_msgs::Imu& msg)
  {
    Eigen::Vector3d lin_acc(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    Eigen::Vector3d ang_vel(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

    return mars::IMUMeasurementType(lin_acc, ang_vel);
  }

  static inline mars_ros::ExtCoreState ExtCoreStateToMsg(const double& t, const mars::CoreStateType& core_state)
  {
    mars_ros::ExtCoreState state_msg;
    state_msg.header.stamp.fromSec(t);

    state_msg.p_wi.x = core_state.p_wi_(0);
    state_msg.p_wi.y = core_state.p_wi_(1);
    state_msg.p_wi.z = core_state.p_wi_(2);

    state_msg.v_wi.x = core_state.v_wi_(0);
    state_msg.v_wi.y = core_state.v_wi_(1);
    state_msg.v_wi.z = core_state.v_wi_(2);

    const Eigen::Vector4d q_wi = core_state.q_wi_.coeffs();  // xyzw
    state_msg.q_wi.x = q_wi(0);
    state_msg.q_wi.y = q_wi(1);
    state_msg.q_wi.z = q_wi(2);
    state_msg.q_wi.w = q_wi(3);

    state_msg.b_w.x = core_state.b_w_(0);
    state_msg.b_w.y = core_state.b_w_(1);
    state_msg.b_w.z = core_state.b_w_(2);

    state_msg.b_a.x = core_state.b_a_(0);
    state_msg.b_a.y = core_state.b_a_(1);
    state_msg.b_a.z = core_state.b_a_(2);

    state_msg.FRAME_TYPE = 1;
    state_msg.QUATERNION_TYPE = 1;

    return state_msg;
  }

  static inline mars_ros::ExtCoreStateLite ExtCoreStateLiteToMsg(const double& t, const mars::CoreStateType& core_state)
  {
    mars_ros::ExtCoreStateLite state_msg;
    state_msg.header.stamp.fromSec(t);

    state_msg.p_wi.x = core_state.p_wi_(0);
    state_msg.p_wi.y = core_state.p_wi_(1);
    state_msg.p_wi.z = core_state.p_wi_(2);

    state_msg.v_wi.x = core_state.v_wi_(0);
    state_msg.v_wi.y = core_state.v_wi_(1);
    state_msg.v_wi.z = core_state.v_wi_(2);

    const Eigen::Vector4d q_wi = core_state.q_wi_.coeffs();  // xyzw
    state_msg.q_wi.x = q_wi(0);
    state_msg.q_wi.y = q_wi(1);
    state_msg.q_wi.z = q_wi(2);
    state_msg.q_wi.w = q_wi(3);

    state_msg.FRAME_TYPE = 1;
    state_msg.QUATERNION_TYPE = 1;

    return state_msg;
  }

  static inline geometry_msgs::PoseStamped ExtCoreStateToPoseMsg(const double& t, const mars::CoreStateType& core_state)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);

    pose_msg.pose.position.x = core_state.p_wi_(0);
    pose_msg.pose.position.y = core_state.p_wi_(1);
    pose_msg.pose.position.z = core_state.p_wi_(2);

    const Eigen::Vector4d q_wi = core_state.q_wi_.coeffs();  // xyzw
    pose_msg.pose.orientation.x = q_wi(0);
    pose_msg.pose.orientation.y = q_wi(1);
    pose_msg.pose.orientation.z = q_wi(2);
    pose_msg.pose.orientation.w = q_wi(3);
    return pose_msg;
  }

  static inline mars::PositionMeasurementType PointMsgToPositionMeas(const geometry_msgs::PointStamped& msg)
  {
    const Eigen::Vector3d position(msg.point.x, msg.point.y, msg.point.z);

    return mars::PositionMeasurementType(position);
  }

  static inline mars::PositionMeasurementType PoseMsgToPositionMeas(const geometry_msgs::PoseStamped& msg)
  {
    const Eigen::Vector3d position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    return mars::PositionMeasurementType(position);
  }

  static inline mars::PositionMeasurementType PoseWithCovMsgToPositionMeas(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    const Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    return mars::PositionMeasurementType(position);
  }

  static inline mars::PositionMeasurementType TransformMsgToPositionMeas(const geometry_msgs::TransformStamped& msg)
  {
    const Eigen::Vector3d position(msg.transform.translation.x, msg.transform.translation.y,
                                   msg.transform.translation.z);

    return mars::PositionMeasurementType(position);
  }

  static inline mars::PositionMeasurementType OdomMsgToPositionMeas(const nav_msgs::Odometry& msg)
  {
    const Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    return mars::PositionMeasurementType(position);
  }

  static inline geometry_msgs::PoseWithCovarianceStamped
  PositionStateToPoseWithCovMsg(const double& t, const mars::PositionSensorStateType& position_state)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);

    pose_msg.pose.pose.position.x = position_state.p_ip_(0);
    pose_msg.pose.pose.position.y = position_state.p_ip_(1);
    pose_msg.pose.pose.position.z = position_state.p_ip_(2);

    const Eigen::Vector4d quat_identity(0, 0, 0, 1);  // xyzw
    pose_msg.pose.pose.orientation.x = quat_identity(0);
    pose_msg.pose.pose.orientation.y = quat_identity(1);
    pose_msg.pose.pose.orientation.z = quat_identity(2);
    pose_msg.pose.pose.orientation.w = quat_identity(3);
    return pose_msg;
  }

  static inline geometry_msgs::PoseWithCovarianceStamped
  PoseStateToPoseWithCovMsg(const double& t, const mars::PoseSensorStateType& pose_state)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);

    pose_msg.pose.pose.position.x = pose_state.p_ip_(0);
    pose_msg.pose.pose.position.y = pose_state.p_ip_(1);
    pose_msg.pose.pose.position.z = pose_state.p_ip_(2);

    const Eigen::Vector4d q_ip = pose_state.q_ip_.coeffs();  // xyzw
    pose_msg.pose.pose.orientation.x = q_ip(0);
    pose_msg.pose.pose.orientation.y = q_ip(1);
    pose_msg.pose.pose.orientation.z = q_ip(2);
    pose_msg.pose.pose.orientation.w = q_ip(3);
    return pose_msg;
  }

  static inline geometry_msgs::PoseStamped PoseStateToPoseMsg(const double& t,
                                                              const mars::PoseSensorStateType& pose_state)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);

    pose_msg.pose.position.x = pose_state.p_ip_(0);
    pose_msg.pose.position.y = pose_state.p_ip_(1);
    pose_msg.pose.position.z = pose_state.p_ip_(2);

    const Eigen::Vector4d q_ip = pose_state.q_ip_.coeffs();  // xyzw
    pose_msg.pose.orientation.x = q_ip(0);
    pose_msg.pose.orientation.y = q_ip(1);
    pose_msg.pose.orientation.z = q_ip(2);
    pose_msg.pose.orientation.w = q_ip(3);
    return pose_msg;
  }

  static inline mars::PoseMeasurementType PoseMsgToPoseMeas(const geometry_msgs::PoseStamped& msg)
  {
    const Eigen::Vector3d position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    const Eigen::Quaterniond orientation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                         msg.pose.orientation.z);
    return mars::PoseMeasurementType(position, orientation);
  }

  static inline mars::PoseMeasurementType PoseWithCovMsgToPoseMeas(const geometry_msgs::PoseWithCovarianceStamped& msg)
  {
    const Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    const Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    return mars::PoseMeasurementType(position, orientation);
  }

  static inline mars::PoseMeasurementType TransformMsgToPoseMeas(const geometry_msgs::TransformStamped& msg)
  {
    const Eigen::Vector3d position(msg.transform.translation.x, msg.transform.translation.y,
                                   msg.transform.translation.z);
    const Eigen::Quaterniond orientation(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
                                         msg.transform.rotation.z);
    return mars::PoseMeasurementType(position, orientation);
  }

  static inline mars::PoseMeasurementType OdomMsgToPoseMeas(const nav_msgs::Odometry& msg)
  {
    const Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    const Eigen::Quaterniond orientation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    return mars::PoseMeasurementType(position, orientation);
  }

  static inline mars::GpsMeasurementType NavSatFixMsgToGpsMeas(const sensor_msgs::NavSatFix& msg)
  {
    return mars::GpsMeasurementType(msg.latitude, msg.longitude, msg.altitude);
  }

  static inline geometry_msgs::PoseWithCovarianceStamped GpsStateToMsg(const double& t,
                                                                       const mars::GpsSensorStateType& gps_state)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);

    pose_msg.pose.pose.position.x = gps_state.p_ig_(0);
    pose_msg.pose.pose.position.y = gps_state.p_ig_(1);
    pose_msg.pose.pose.position.z = gps_state.p_ig_(2);

    // Return unit quaternion
    pose_msg.pose.pose.orientation.x = 0;
    pose_msg.pose.pose.orientation.y = 0;
    pose_msg.pose.pose.orientation.z = 0;
    pose_msg.pose.pose.orientation.w = 1;
    return pose_msg;
  }
};

#endif  // MARS_MSG_CONV_H
