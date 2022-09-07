// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
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
#include <mars_ros/ExtCoreState.h>
#include <mars_ros/ExtCoreStateLite.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>

using namespace mars;

MarsWrapperGps::MarsWrapperGps(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperGps::configCallback, this, _1, _2))
  , p_wi_init_(0, 0, 0)
  , q_wi_init_(Eigen::Quaterniond::Identity())
  , m_sett_(nh)
{
  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperGps::initServiceCallback, this);

  std::cout << "Setup framework components" << std::endl;

  // Framework components
  imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr_ = std::make_shared<mars::CoreState>();
  core_states_sptr_.get()->set_initial_covariance(m_sett_.core_init_cov_p_, m_sett_.core_init_cov_v_,
                                                  m_sett_.core_init_cov_q_, m_sett_.core_init_cov_bw_,
                                                  m_sett_.core_init_cov_ba_);

  core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
  core_logic_ = mars::CoreLogic(core_states_sptr_);
  core_logic_.buffer_.set_max_buffer_size(m_sett_.buffer_size_);

  core_logic_.verbose_ = m_sett_.verbose_output_;
  core_logic_.verbose_out_of_order_ = m_sett_.verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = m_sett_.discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(
      Eigen::Vector3d(m_sett_.g_rate_noise_, m_sett_.g_rate_noise_, m_sett_.g_rate_noise_),
      Eigen::Vector3d(m_sett_.g_bias_noise_, m_sett_.g_bias_noise_, m_sett_.g_bias_noise_),
      Eigen::Vector3d(m_sett_.a_noise_, m_sett_.a_noise_, m_sett_.a_noise_),
      Eigen::Vector3d(m_sett_.a_bias_noise_, m_sett_.a_bias_noise_, m_sett_.a_bias_noise_));

  // Sensors
  gps1_sensor_sptr_ = std::make_shared<mars::GpsSensorClass>("Gps1", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> gps_meas_std;
    gps_meas_std << m_sett_.gps1_pos_meas_noise_, m_sett_.gps1_vel_meas_noise_;
    gps1_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    GpsSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett_.gps1_cal_ig_);

    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett_.gps1_state_init_cov_, 1e-20, 1e-20, 1e-20, 1e-20, 1e-20, 1e-20;
    gps_calibration.sensor_cov_ = gps_cov;

    gps1_sensor_sptr_->set_initial_calib(std::make_shared<GpsSensorData>(gps_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    gps1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // Subscriber
  sub_imu_measurement_ =
      nh.subscribe("imu_in", m_sett_.sub_imu_cb_buffer_size_, &MarsWrapperGps::ImuMeasurementCallback, this);
  sub_gps1_measurement_ =
      nh.subscribe("gps1_in", m_sett_.sub_sensor_cb_buffer_size_, &MarsWrapperGps::Gps1MeasurementCallback, this);

  // Publisher
  pub_ext_core_state_ = nh.advertise<mars_ros::ExtCoreState>("core_ext_state_out", m_sett_.pub_cb_buffer_size_);
  pub_ext_core_state_lite_ =
      nh.advertise<mars_ros::ExtCoreStateLite>("core_ext_state_lite_out", m_sett_.pub_cb_buffer_size_);
  pub_core_pose_state_ = nh.advertise<geometry_msgs::PoseStamped>("core_pose_state_out", m_sett_.pub_cb_buffer_size_);
  pub_core_odom_state_ = nh.advertise<nav_msgs::Odometry>("core_odom_state_out", m_sett_.pub_cb_buffer_size_);
  if (m_sett_.pub_path_)
  {
    pub_core_path_ = nh.advertise<nav_msgs::Path>("core_states_path", m_sett_.pub_cb_buffer_size_);
  }

  pub_gps1_state_ =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps1_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_gps1_enu_odom_ = nh.advertise<nav_msgs::Odometry>("gps1_enu", m_sett_.pub_cb_buffer_size_);

  // Set initial orientation if manual setting is enabled
  if (this->m_sett_.enable_manual_yaw_init_)
  {
    const double yaw = m_sett_.yaw_init_deg_ * (M_PI / 180);
    Eigen::Matrix3d R_wi;
    R_wi << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    q_wi_init_ = Eigen::Quaterniond(R_wi);

    std::cout << "Manual yaw initialization: " << yaw * (180 / M_PI) << "\n" << std::endl;
  }
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
  if (m_sett_.bypass_init_service_)
  {
    ROS_INFO_STREAM("Initialized filter Bypass ROS Service Response");
  }
  else
  {
    init();
    ROS_INFO_STREAM("Initialized filter trough ROS Service");
  }

  response.success = true;
  return true;
}

void MarsWrapperGps::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
{
  // Config parameter overwrite
  m_sett_.publish_on_propagation_ = config.pub_on_prop;
  core_logic_.verbose_ = config.verbose;
  m_sett_.verbose_output_ = config.verbose;
  m_sett_.use_ros_time_now_ = config.use_ros_time_now;

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

  if (m_sett_.use_ros_time_now_)
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
  const bool valid_update = core_logic_.ProcessMeasurement(imu_sensor_sptr_, timestamp, data);

  // Initialize the first time at which the propagation sensor occures
  if (!core_logic_.core_is_initialized_)
  {
    core_logic_.Initialize(p_wi_init_, q_wi_init_);
  }

  if (m_sett_.publish_on_propagation_ && valid_update)
  {
    this->RunCoreStatePublisher();
  }
}

void MarsWrapperGps::Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas)
{
  Time timestamp;

  if (m_sett_.use_ros_time_now_)
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

  // Publish GPS ENU as Odometry
  if (m_sett_.publish_gps_enu_)
  {
    Eigen::Vector3d gps_enu(gps1_sensor_sptr_->gps_conversion_.get_enu(gps_meas.coordinates_));
    pub_gps1_enu_odom_.publish(MarsMsgConv::EigenVec3dToOdomMsg(timestamp.get_seconds(), gps_enu));
  }

  // Publish latest sensor state
  mars::BufferEntryType latest_sensor_state;
  const bool valid_state = core_logic_.buffer_.get_latest_sensor_handle_state(gps1_sensor_sptr_, &latest_sensor_state);

  if (!valid_state)
  {
    return;
  }

  mars::GpsSensorStateType gps_sensor_state = gps1_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);

  pub_gps1_state_.publish(MarsMsgConv::GpsStateToMsg(latest_sensor_state.timestamp_.get_seconds(), gps_sensor_state));
}

void MarsWrapperGps::RunCoreStatePublisher()
{
  mars::BufferEntryType latest_state;
  const bool valid_state = core_logic_.buffer_.get_latest_state(&latest_state);

  if (!valid_state)
  {
    return;
  }

  mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

  if (m_sett_.pub_cov_)
  {
    mars::CoreStateMatrix cov = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->cov_;
    pub_ext_core_state_.publish(
        MarsMsgConv::ExtCoreStateToMsgCov(latest_state.timestamp_.get_seconds(), latest_core_state, cov));
  }
  else
  {
    pub_ext_core_state_.publish(
        MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  }

  pub_ext_core_state_lite_.publish(
      MarsMsgConv::ExtCoreStateLiteToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_pose_state_.publish(
      MarsMsgConv::ExtCoreStateToPoseMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_odom_state_.publish(
      MarsMsgConv::ExtCoreStateToOdomMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  if (m_sett_.pub_path_)
  {
    pub_core_path_.publish(
        path_generator_.ExtCoreStateToPathMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  }
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
}
