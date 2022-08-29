// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "mars_wrapper_gps.h"
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars_msg_conv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include "mars_msg_conv.h"

using namespace mars;

MarsWrapperGps::MarsWrapperGps(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperGps::configCallback, this, _1, _2))
  , p_wi_init_(0, 0, 0)
  , q_wi_init_(Eigen::Quaterniond::Identity())
{
  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperGps::initServiceCallback, this);

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

  core_states_sptr_->set_noise_std(
      Eigen::Vector3d(0.0024, 0.0024, 0.0024), Eigen::Vector3d(1.319e-5, 1.319e-5, 1.319e-5),
      Eigen::Vector3d(0.1071, 0.1071, 0.1071), Eigen::Vector3d(2.2246e-4, 2.2246e-4, 2.2246e-4));

  // Sensors
  gps1_sensor_sptr_ = std::make_shared<mars::GpsSensorClass>("Gps1", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> gps_meas_std;
    gps_meas_std << 0.5, 0.5, 1.0;
    gps1_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    GpsSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << 1e-2, 1e-2, 1e-2, 1e-16, 1e-16, 1e-16, 1e-16, 1e-16, 1e-16;
    gps_calibration.sensor_cov_ = gps_cov;
    gps1_sensor_sptr_->set_initial_calib(std::make_shared<GpsSensorData>(gps_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    gps1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // Subscriber
  sub_imu_measurement_ = nh.subscribe("imu_in", sub_imu_cb_buffer_size_, &MarsWrapperGps::ImuMeasurementCallback, this);
  sub_gps1_measurement_ =
      nh.subscribe("gps_in", sub_sensor_cb_buffer_size_, &MarsWrapperGps::GpsMeasurementCallback, this);

  // Publisher
  pub_ext_core_state_ = nh.advertise<mars_ros::ExtCoreState>("core_ext_state_out", pub_cb_buffer_size_);
  pub_ext_core_state_lite_ = nh.advertise<mars_ros::ExtCoreStateLite>("core_ext_state_lite_out", pub_cb_buffer_size_);
  pub_core_pose_state_ = nh.advertise<geometry_msgs::PoseStamped>("core_pose_state_out", pub_cb_buffer_size_);
  pub_core_odom_state_ = nh.advertise<nav_msgs::Odometry>("core_odom_state_out", pub_cb_buffer_size_);
  pub_gps_state_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps_cal_state_out", pub_cb_buffer_size_);
}

bool MarsWrapperGps::init()
{
  core_logic_.core_is_initialized_ = false;
  core_logic_.buffer_.ResetBufferData();
  gps1_sensor_sptr_->is_initialized_ = false;
  common_gps_ref_is_set_ = false;

  return true;
}

bool MarsWrapperGps::initServiceCallback(std_srvs::SetBool::Request& /*request*/, std_srvs::SetBool::Response& response)
{
  init();
  ROS_INFO_STREAM("Initialized filter trough ROS Service");

  response.success = true;
  return true;
}

void MarsWrapperGps::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
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

void MarsWrapperGps::set_common_gps_reference(const GpsCoordinates& reference)
{
  if (!common_gps_ref_is_set_)
  {
    std::cout << "Setting the common GPS reference to: \n" << reference << std::endl;

    gps1_sensor_sptr_->set_gps_reference_coordinates(reference);

    common_gps_ref_is_set_ = true;
  }
}

void MarsWrapperGps::ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas)
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
    mars::BufferEntryType latest_state;
    core_logic_.buffer_.get_latest_state(&latest_state);
    mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

    pub_ext_core_state_.publish(
        MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  }
}

void MarsWrapperGps::GpsMeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas)
{
  Time timestamp;

  if (use_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(meas->header.stamp.toSec());
  }

  // Map the measurement to the mars sensor type
  GpsMeasurementType gps_meas = MarsMsgConv::NavSatFixMsgToGpsMeas(*meas);
  set_common_gps_reference(gps_meas.coordinates_);

  GpsMeasurementUpdate(gps1_sensor_sptr_, gps_meas, timestamp);
}

void MarsWrapperGps::RunCoreStatePublisher()
{
  mars::BufferEntryType latest_state;
  core_logic_.buffer_.get_latest_state(&latest_state);
  mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

  pub_ext_core_state_.publish(MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  pub_ext_core_state_lite_.publish(
      MarsMsgConv::ExtCoreStateLiteToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_pose_state_.publish(
      MarsMsgConv::ExtCoreStateToPoseMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_odom_state_.publish(
      MarsMsgConv::ExtCoreStateToOdomMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
}

void MarsWrapperGps::GpsMeasurementUpdate(std::shared_ptr<mars::GpsSensorClass> sensor_sptr,
                                          const GpsMeasurementType& gps_meas, const Time& timestamp)
{
  // TMP feedback init pose
  // p_wi_init_ = pose_meas.position_;
  // q_wi_init_ = pose_meas.orientation_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<GpsMeasurementType>(gps_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp, data))
  {
    return;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  // Publish latest sensor state
  mars::BufferEntryType latest_sensor_state;
  const bool valid_state = core_logic_.buffer_.get_latest_sensor_handle_state(sensor_sptr, &latest_sensor_state);

  if (!valid_state)
  {
    return;
  }

  mars::GpsSensorStateType gps_sensor_state = sensor_sptr.get()->get_state(latest_sensor_state.data_.sensor_);
  pub_gps_state_.publish(MarsMsgConv::GpsStateToMsg(latest_sensor_state.timestamp_.get_seconds(), gps_sensor_state));
}
