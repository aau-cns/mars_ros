// Copyright (C) 2022 Christian Brommer and Martin Scheiber,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>
// and <martin.scheiber@ieee.org>.

#ifndef MARS_MSG_CONV_H
#define MARS_MSG_CONV_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mars/buffer.h>
#include <mars/core_state.h>
#include <mars/sensors/gps/gps_conversion.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_state_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_sensor_state_type.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/mag/mag_measurement_type.h>
#include <mars/sensors/mag/mag_sensor_state_type.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_state_type.h>
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/sensors/position/position_sensor_state_type.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/pressure/pressure_sensor_state_type.h>
#include <mars/sensors/vision/vision_measurement_type.h>
#include <mars/sensors/vision/vision_sensor_state_type.h>
#include <mars/type_definitions/core_state_type.h>
#include <mars_ros/ExtCoreState.h>
#include <mars_ros/ExtCoreStateLite.h>
#include <mars_ros/VisionSensorState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>

#include <Eigen/Dense>

#define DEFAULT_FRAME_ID "world"

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

  static inline mars_ros::ExtCoreState ExtCoreStateToMsg(const double& t, const mars::CoreStateType& core_state,
                                                         const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    mars_ros::ExtCoreState state_msg;
    state_msg.header.stamp.fromSec(t);
    state_msg.header.frame_id = frame_id;

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

  static inline mars_ros::ExtCoreState ExtCoreStateToMsgCov(const double& t, const mars::CoreStateType& core_state,
                                                            const mars::CoreStateMatrix state_cov,
                                                            const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    // Map States
    mars_ros::ExtCoreState state_msg(ExtCoreStateToMsg(t, core_state, frame_id));

    // Map Covariance
    std::copy(state_cov.data(), state_cov.data() + state_cov.size(), state_msg.cov.begin());
    return state_msg;
  }

  static inline mars_ros::ExtCoreStateLite ExtCoreStateLiteToMsg(const double& t, const mars::CoreStateType& core_state,
                                                                 const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    mars_ros::ExtCoreStateLite state_msg;
    state_msg.header.stamp.fromSec(t);
    state_msg.header.frame_id = frame_id;

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

  static inline geometry_msgs::PoseStamped ExtCoreStateToPoseMsg(const double& t, const mars::CoreStateType& core_state,
                                                                 const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

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

  static inline nav_msgs::Odometry ExtCoreStateToOdomMsg(const double& t, const mars::CoreStateType& core_state,
                                                         const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp.fromSec(t);
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = "map";
    odom_msg.pose.pose.position.x = core_state.p_wi_.x();
    odom_msg.pose.pose.position.y = core_state.p_wi_.y();
    odom_msg.pose.pose.position.z = core_state.p_wi_.z();

    odom_msg.pose.pose.orientation.w = core_state.q_wi_.w();
    odom_msg.pose.pose.orientation.x = core_state.q_wi_.x();
    odom_msg.pose.pose.orientation.y = core_state.q_wi_.y();
    odom_msg.pose.pose.orientation.z = core_state.q_wi_.z();

    odom_msg.twist.twist.linear.x = core_state.v_wi_.x();
    odom_msg.twist.twist.linear.y = core_state.v_wi_.y();
    odom_msg.twist.twist.linear.z = core_state.v_wi_.z();

    return odom_msg;
  }

  static inline nav_msgs::Path BufferCoreStateToPathMsg(const double& t, const mars::Buffer& buffer,
                                                        const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    /// \attention this iterates through the full buffer --> performance loss!
    /// use the MarsPathGen class instead!

    nav_msgs::Path path_msg;
    path_msg.header.stamp.fromSec(t);
    path_msg.header.frame_id = frame_id;
    path_msg.poses.clear();

    /// \todo(scm): this functionality to get all states could be part of the buffer
    for (int idx = 0; idx < buffer.get_length(); ++idx)
    {
      mars::BufferEntryType buffer_entry;
      if (!buffer.get_entry_at_idx(idx, &buffer_entry))
      {
        break;
      }

      // add core state to path msg if vaild
      if (buffer_entry.IsState())
      {
        mars::CoreStateType buffer_core_state = static_cast<mars::CoreType*>(buffer_entry.data_.core_.get())->state_;
        path_msg.poses.push_back(
            ExtCoreStateToPoseMsg(buffer_entry.timestamp_.get_seconds(), buffer_core_state, frame_id));
      }
    }

    return path_msg;
  }

  static inline nav_msgs::Odometry EigenVec3dToOdomMsg(const double& t, const Eigen::Vector3d& position,
                                                       const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp.fromSec(t);
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = "map";
    odom_msg.pose.pose.position.x = position[0];
    odom_msg.pose.pose.position.y = position[1];
    odom_msg.pose.pose.position.z = position[2];

    odom_msg.pose.pose.orientation.w = 1;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = 0;

    return odom_msg;
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

  static inline mars::PositionMeasurementType
  PoseWithCovMsgToPositionMeas(const geometry_msgs::PoseWithCovarianceStamped& msg)
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
  PositionStateToPoseWithCovMsg(const double& t, const mars::PositionSensorStateType& position_state,
                                const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

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

  static inline geometry_msgs::PoseWithCovarianceStamped PoseStateToPoseWithCovMsg(
      const double& t, const mars::PoseSensorStateType& pose_state, const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

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
                                                              const mars::PoseSensorStateType& pose_state,
                                                              const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

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

  static inline mars::VisionMeasurementType PoseMsgToVisionMeas(const geometry_msgs::PoseStamped& msg)
  {
    const Eigen::Vector3d position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    const Eigen::Quaterniond orientation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                         msg.pose.orientation.z);
    return mars::VisionMeasurementType(position, orientation);
  }

  static inline mars::GpsMeasurementType NavSatFixMsgToGpsMeas(const sensor_msgs::NavSatFix& msg)
  {
    return mars::GpsMeasurementType(msg.latitude, msg.longitude, msg.altitude);
  }

  static inline mars::GpsVelMeasurementType NavSatTwistWithCovMsgToGpsVelMeas(
      const sensor_msgs::NavSatFix& msg_coord, const geometry_msgs::TwistWithCovarianceStamped& msg_vel)
  {
    return mars::GpsVelMeasurementType(msg_coord.latitude, msg_coord.longitude, msg_coord.altitude,
                                       msg_vel.twist.twist.linear.x, msg_vel.twist.twist.linear.y,
                                       msg_vel.twist.twist.linear.z);
  }

  static inline mars::GpsVelMeasurementType NavSatTwistMsgToGpsVelMeas(const sensor_msgs::NavSatFix& msg_coord,
                                                                       const geometry_msgs::TwistStamped& msg_vel)
  {
    return mars::GpsVelMeasurementType(msg_coord.latitude, msg_coord.longitude, msg_coord.altitude,
                                       msg_vel.twist.linear.x, msg_vel.twist.linear.y, msg_vel.twist.linear.z);
  }

  static inline geometry_msgs::PoseWithCovarianceStamped GpsStateToMsg(const double& t,
                                                                       const mars::GpsSensorStateType& gps_state,
                                                                       const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

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

  static inline geometry_msgs::PoseWithCovarianceStamped GpsVelStateToMsg(
      const double& t, const mars::GpsVelSensorStateType& gps_state, const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

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

  static inline mars::MagMeasurementType MagMsgToMagMeas(const sensor_msgs::MagneticField& msg)
  {
    const Eigen::Vector3d mag_vector(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
    return mars::MagMeasurementType(mag_vector);
  }

  static inline geometry_msgs::PoseWithCovarianceStamped MagStateToMsg(const double& t,
                                                                       const mars::MagSensorStateType& mag_state,
                                                                       const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.pose.position.x = mag_state.mag_(0);
    pose_msg.pose.pose.position.y = mag_state.mag_(1);
    pose_msg.pose.pose.position.z = mag_state.mag_(2);

    pose_msg.pose.pose.orientation.x = mag_state.q_im_.x();
    pose_msg.pose.pose.orientation.y = mag_state.q_im_.y();
    pose_msg.pose.pose.orientation.z = mag_state.q_im_.z();
    pose_msg.pose.pose.orientation.w = mag_state.q_im_.w();
    return pose_msg;
  }

  static inline mars::PressureMeasurementType FluidPressureMsgtoPressureMeas(const sensor_msgs::FluidPressure& msg,
                                                                             const double& temperature)
  {
    return mars::PressureMeasurementType(msg.fluid_pressure, temperature);
  }

  static inline mars::PressureMeasurementType
  FluidPressureMsgtoPressureMeas(const sensor_msgs::FluidPressure& msg_press, const sensor_msgs::Temperature& msg_temp)
  {
    return mars::PressureMeasurementType(msg_press.fluid_pressure, msg_temp.temperature);
  }

  static inline geometry_msgs::PoseStamped PressureStateToMsg(const double& t,
                                                              const mars::PressureSensorStateType& pressure_state,
                                                              const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.fromSec(t);
    pose_msg.header.frame_id = frame_id;

    pose_msg.pose.position.x = pressure_state.p_ip_(0);
    pose_msg.pose.position.y = pressure_state.p_ip_(1);
    pose_msg.pose.position.z = pressure_state.p_ip_(2);

    // Return unit quaternion
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 0;
    pose_msg.pose.orientation.w = 1;
    return pose_msg;
  }

  static inline geometry_msgs::Vector3Stamped EigenVec3dToVec3Msg(const double& t, const Eigen::Vector3d& vec,
                                                                  const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    geometry_msgs::Vector3Stamped vec_msg;
    vec_msg.header.stamp.fromSec(t);
    vec_msg.header.frame_id = frame_id;

    vec_msg.vector.x = vec(0);
    vec_msg.vector.y = vec(1);
    vec_msg.vector.z = vec(2);
    return vec_msg;
  }

  static inline mars_ros::VisionSensorState VisionStateToMsg(const double& t,
                                                             const mars::VisionSensorStateType& vision_state,
                                                             const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    mars_ros::VisionSensorState vision_msg;
    vision_msg.header.stamp.fromSec(t);
    vision_msg.header.frame_id = frame_id;

    vision_msg.p_ic.x = vision_state.p_ic_(0);
    vision_msg.p_ic.y = vision_state.p_ic_(1);
    vision_msg.p_ic.z = vision_state.p_ic_(2);
    vision_msg.q_ic.x = vision_state.q_ic_.x();
    vision_msg.q_ic.y = vision_state.q_ic_.y();
    vision_msg.q_ic.z = vision_state.q_ic_.z();
    vision_msg.q_ic.w = vision_state.q_ic_.w();

    vision_msg.p_vw.x = vision_state.p_vw_(0);
    vision_msg.p_vw.y = vision_state.p_vw_(1);
    vision_msg.p_vw.z = vision_state.p_vw_(2);
    vision_msg.q_vw.x = vision_state.q_vw_.x();
    vision_msg.q_vw.y = vision_state.q_vw_.y();
    vision_msg.q_vw.z = vision_state.q_vw_.z();
    vision_msg.q_vw.w = vision_state.q_vw_.w();

    vision_msg.lambda = vision_state.lambda_;

    return vision_msg;
  }
};

///
/// \brief The MarsPathGen class Conversion and storage between MaRS core state to ROS Path message
///
class MarsPathGen
{
private:
  std::vector<geometry_msgs::PoseStamped> path_poses_;
  const size_t max_path_length_{ 2000 };

public:
  MarsPathGen() = default;
  MarsPathGen(const size_t& max_path_length) : max_path_length_(max_path_length){};

  inline nav_msgs::Path ExtCoreStateToPathMsg(const double& t, const mars::CoreStateType& core_state,
                                              const std::string& frame_id = DEFAULT_FRAME_ID)
  {
    nav_msgs::Path path_msg;
    path_msg.header.stamp.fromSec(t);
    path_msg.header.frame_id = frame_id;
    path_msg.poses.clear();

    // add pose and check size
    path_poses_.push_back(MarsMsgConv::ExtCoreStateToPoseMsg(t, core_state, frame_id));
    if (path_poses_.size() > max_path_length_)
      path_poses_.erase(path_poses_.begin());
    path_msg.poses = path_poses_;

    return path_msg;
  }
};

#endif  // MARS_MSG_CONV_H
