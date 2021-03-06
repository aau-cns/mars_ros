// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "mars_wrapper_position.h"
#include "mars_msg_conv.h"

#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/position/position_measurement_type.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars_msg_conv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>

using namespace mars;

MarsWrapperPosition::MarsWrapperPosition(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperPosition::configCallback, this, _1, _2))
  , p_wi_init_(0, 0, 0)
  , q_wi_init_(Eigen::Quaterniond::Identity())
{
  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperPosition::initServiceCallback, this);

  // Read parameter
  publish_on_propagation_ = nh.param<bool>("pub_on_prop", publish_on_propagation_);
  use_ros_time_now_ = nh.param<bool>("use_ros_time_now", use_ros_time_now_);
  verbose_output_ = nh.param<bool>("verbose", verbose_output_);
  verbose_ooo_ = nh.param<bool>("verbose_out_of_order", verbose_ooo_);
  discard_ooo_prop_meas_ = nh.param<bool>("discard_ooo_prop_meas", discard_ooo_prop_meas_);

  pub_cb_buffer_size_ = uint32_t(nh.param<int>("pub_cb_buffer_size", int(pub_cb_buffer_size_)));
  sub_imu_cb_buffer_size_ = uint32_t(nh.param<int>("sub_imu_cb_buffer_size", int(sub_imu_cb_buffer_size_)));
  sub_sensor_cb_buffer_size_ = uint32_t(nh.param<int>("sub_sensor_cb_buffer_size", int(sub_sensor_cb_buffer_size_)));

  std::cout << "Setup framework components" << std::endl;

  // Framework components
  imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr_ = std::make_shared<mars::CoreState>();
  core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
  core_logic_ = mars::CoreLogic(core_states_sptr_);
  core_logic_.buffer_.set_max_buffer_size(800);

  core_logic_.verbose_ = verbose_output_;
  core_logic_.verbose_out_of_order_ = verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(Eigen::Vector3d(0.013, 0.013, 0.013), Eigen::Vector3d(0.0013, 0.0013, 0.0013),
                                   Eigen::Vector3d(0.083, 0.083, 0.083), Eigen::Vector3d(0.0083, 0.0083, 0.0083));

  // Sensors
  position1_sensor_sptr_ = std::make_shared<mars::PositionSensorClass>("Position1", core_states_sptr_);
  Eigen::Matrix<double, 6, 1> position_meas_std;
  position_meas_std << 0.05, 0.05, 0.05;
  position1_sensor_sptr_->R_ = position_meas_std.cwiseProduct(position_meas_std);

  PositionSensorData position_calibration;
  position_calibration.state_.p_ip_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Matrix<double, 3, 3> position_cov;
  position_cov.setZero();
  position_cov.diagonal() << 0.0025, 0.0025, 0.0025;  // 5cm
  position_calibration.sensor_cov_ = position_cov;
  position1_sensor_sptr_->set_initial_calib(std::make_shared<PositionSensorData>(position_calibration));

  // TODO is set here for now, but will be managed by core logic in later versions
  position1_sensor_sptr_->const_ref_to_nav_ = true;

  // Subscriber
  sub_imu_measurement_ =
      nh.subscribe("imu_in", sub_imu_cb_buffer_size_, &MarsWrapperPosition::ImuMeasurementCallback, this);
  sub_point_measurement_ =
      nh.subscribe("point_in", sub_sensor_cb_buffer_size_, &MarsWrapperPosition::PointMeasurementCallback, this);
  sub_pose_measurement_ =
      nh.subscribe("pose_in", sub_sensor_cb_buffer_size_, &MarsWrapperPosition::PoseMeasurementCallback, this);
  sub_pose_with_cov_measurement_ = nh.subscribe("pose_with_cov_in", sub_sensor_cb_buffer_size_,
                                                &MarsWrapperPosition::PoseWithCovMeasurementCallback, this);
  sub_odom_measurement_ =
      nh.subscribe("odom_in", sub_sensor_cb_buffer_size_, &MarsWrapperPosition::OdomMeasurementCallback, this);
  sub_transform_measurement_ = nh.subscribe("transform_in", sub_sensor_cb_buffer_size_,
                                            &MarsWrapperPosition::TransformMeasurementCallback, this);

  // Publisher
  pub_ext_core_state_ = nh.advertise<mars_ros::ExtCoreState>("core_ext_state_out", pub_cb_buffer_size_);
  pub_ext_core_state_lite_ = nh.advertise<mars_ros::ExtCoreStateLite>("core_ext_state_lite_out", pub_cb_buffer_size_);
  pub_core_pose_state_ = nh.advertise<geometry_msgs::PoseStamped>("core_pose_state_out", pub_cb_buffer_size_);
  pub_position1_state_ = nh.advertise<geometry_msgs::PoseStamped>("position_cal_state_out", pub_cb_buffer_size_);
}

bool MarsWrapperPosition::init()
{
  core_logic_.core_is_initialized_ = false;
  core_logic_.buffer_.ResetBufferData();
  position1_sensor_sptr_->is_initialized_ = false;

  return true;
}

bool MarsWrapperPosition::initServiceCallback(std_srvs::SetBool::Request& /*request*/, std_srvs::SetBool::Response& res)
{
  init();
  res.success = true;
  ROS_INFO_STREAM("Initialized filter trough ROS Service");

  return true;
}

void MarsWrapperPosition::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
{
  // Config parameter overwrite
  publish_on_propagation_ = config.pub_on_prop;
  core_logic_.verbose_ = config.verbose;
  verbose_output_ = config.verbose;
  use_ros_time_now_ = config.use_ros_time_now;

  if (config.initialize)
  {
    init();
    ROS_INFO_STREAM("Initialized filter trough Reconfigure GUI");
  }

  config.initialize = false;
}

void MarsWrapperPosition::ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas)
{
  // Map the measutement to the mars type
  Time timestamp;

  if (use_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(meas->header.stamp.toSec());
  }

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<IMUMeasurementType>(MarsMsgConv::ImuMsgToImuMeas(*meas)));

  // Call process measurement
  core_logic_.ProcessMeasurement(imu_sensor_sptr_, timestamp, data);

  // Initialize the first time at which the propagation sensor occures
  if (!core_logic_.core_is_initialized_)
  {
    core_logic_.Initialize(p_wi_init_, q_wi_init_);
  }

  if (publish_on_propagation_)
  {
    this->RunCoreStatePublisher();
  }
}

void MarsWrapperPosition::PointMeasurementCallback(const geometry_msgs::PointStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PositionMeasurementType position_meas = MarsMsgConv::PointMsgToPositionMeas(*meas);
  PositionMeasurementUpdate(position1_sensor_sptr_, position_meas, timestamp);
}

void MarsWrapperPosition::PoseMeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PositionMeasurementType position_meas = MarsMsgConv::PoseMsgToPositionMeas(*meas);
  PositionMeasurementUpdate(position1_sensor_sptr_, position_meas, timestamp);
}

void MarsWrapperPosition::PoseWithCovMeasurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PositionMeasurementType position_meas = MarsMsgConv::PoseWithCovMsgToPositionMeas(*meas);
  PositionMeasurementUpdate(position1_sensor_sptr_, position_meas, timestamp);
}

void MarsWrapperPosition::TransformMeasurementCallback(const geometry_msgs::TransformStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PositionMeasurementType position_meas = MarsMsgConv::TransformMsgToPositionMeas(*meas);
  PositionMeasurementUpdate(position1_sensor_sptr_, position_meas, timestamp);
}

void MarsWrapperPosition::OdomMeasurementCallback(const nav_msgs::OdometryConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PositionMeasurementType position_meas = MarsMsgConv::OdomMsgToPositionMeas(*meas);
  PositionMeasurementUpdate(position1_sensor_sptr_, position_meas, timestamp);
}

void MarsWrapperPosition::RunCoreStatePublisher()
{
  mars::BufferEntryType latest_state;
  core_logic_.buffer_.get_latest_state(&latest_state);
  mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

  pub_ext_core_state_.publish(MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  pub_ext_core_state_lite_.publish(
      MarsMsgConv::ExtCoreStateLiteToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_pose_state_.publish(
      MarsMsgConv::ExtCoreStateToPoseMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
}

void MarsWrapperPosition::PositionMeasurementUpdate(std::shared_ptr<mars::PositionSensorClass> sensor_sptr,
                                                    const mars::PositionMeasurementType& position_meas,
                                                    const mars::Time& timestamp)
{
  Time timestamp_corr;

  if (use_ros_time_now_)
  {
    timestamp_corr = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp_corr = timestamp;
  }

  // TMP feedback init position
  p_wi_init_ = position_meas.position_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<PositionMeasurementType>(position_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp_corr, data))
  {
    return;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  // Publish the latest sensor state
  mars::BufferEntryType latest_result;
  core_logic_.buffer_.get_latest_sensor_handle_state(sensor_sptr, &latest_result);
  mars::PositionSensorStateType position_sensor_state = sensor_sptr.get()->get_state(latest_result.data_.sensor_);

  pub_position1_state_.publish(
      MarsMsgConv::PositionStateToPoseWithCovMsg(latest_result.timestamp_.get_seconds(), position_sensor_state));
}
