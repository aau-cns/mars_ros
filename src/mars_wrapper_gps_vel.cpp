// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include "mars_wrapper_gps_vel.h"

#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars_msg_conv.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include "mars_msg_conv.h"

using namespace mars;

MarsWrapperGpsVel::MarsWrapperGpsVel(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperGpsVel::configCallback, this, _1, _2))
  , m_sett(nh)
#if NOT APPROX_TIME_SYNC
  , sync_gps1_meas_(sub_gps1_coord_meas_, sub_gps1_vel_meas_, sub_sensor_cb_buffer_size_)
#else
  , sync_gps1_meas_(ApproxTimePolicy(3), sub_gps1_coord_meas_, sub_gps1_vel_meas_)
#endif

  , p_wi_init_(0.0, 0.0, 0.0)
  , q_wi_init_(Eigen::Quaterniond::Identity())
{
  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperGpsVel::initServiceCallback, this);

  std::cout << "Setup framework components" << std::endl;

  // Framework components
  imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr_ = std::make_shared<mars::CoreState>();
  core_states_sptr_.get()->set_initial_covariance(m_sett.core_init_cov_p_, m_sett.core_init_cov_v_,
                                                  m_sett.core_init_cov_q_, m_sett.core_init_cov_bw_,
                                                  m_sett.core_init_cov_ba_);

  core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
  core_logic_ = mars::CoreLogic(core_states_sptr_);
  core_logic_.buffer_.set_max_buffer_size(800);

  core_logic_.verbose_ = m_sett.verbose_output_;
  core_logic_.verbose_out_of_order_ = m_sett.verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = m_sett.discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(Eigen::Vector3d(m_sett.g_rate_noise_, m_sett.g_rate_noise_, m_sett.g_rate_noise_),
                                   Eigen::Vector3d(m_sett.g_bias_noise_, m_sett.g_bias_noise_, m_sett.g_bias_noise_),
                                   Eigen::Vector3d(m_sett.a_noise_, m_sett.a_noise_, m_sett.a_noise_),
                                   Eigen::Vector3d(m_sett.a_bias_noise_, m_sett.a_bias_noise_, m_sett.a_bias_noise_));

  if (this->m_sett.enable_manual_yaw_init_)
  {
    const double yaw = m_sett.yaw_init_deg_ * (M_PI / 180);
    Eigen::Matrix3d r;
    r << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    q_wi_init_ = Eigen::Quaterniond(r);

    std::cout << "Manual yaw initialization: " << yaw * (180 / M_PI) << "\n" << std::endl;

    mag_init_.set_done();
  }

  // Sensors
  // GPS1
  gps1_sensor_sptr_ = std::make_shared<mars::GpsVelSensorClass>("Gps1", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> gps_meas_std;

    gps_meas_std << m_sett.gps_pos_meas_noise_, m_sett.gps_vel_meas_noise_;

    gps1_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    GpsVelSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett.gps_cal_ig_);
    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett.gps_state_init_cov_, 1e-20, 1e-20, 1e-20, 1e-20, 1e-20, 1e-20;
    gps_calibration.sensor_cov_ = gps_cov;

    gps1_sensor_sptr_->set_initial_calib(std::make_shared<GpsVelSensorData>(gps_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    gps1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // Subscriber
  sub_imu_measurement_ =
      nh.subscribe("imu_in", m_sett.sub_imu_cb_buffer_size_, &MarsWrapperGpsVel::ImuMeasurementCallback, this);

  sub_gps1_coord_meas_.subscribe(nh, "gps1_coord_in", m_sett.sub_sensor_cb_buffer_size_);
  sub_gps1_vel_meas_.subscribe(nh, "gps1_vel_in", m_sett.sub_sensor_cb_buffer_size_);
  sync_gps1_meas_.registerCallback(boost::bind(&MarsWrapperGpsVel::Gps1MeasurementCallback, this, _1, _2));

  sub_mag_measurement_ =
      nh.subscribe("mag1_in", m_sett.sub_sensor_cb_buffer_size_, &MarsWrapperGpsVel::MagMeasurementCallback, this);

  // Publisher
  pub_ext_core_state_ = nh.advertise<mars_ros::ExtCoreState>("core_ext_state_out", m_sett.pub_cb_buffer_size_);
  pub_ext_core_state_lite_ =
      nh.advertise<mars_ros::ExtCoreStateLite>("core_ext_state_lite_out", m_sett.pub_cb_buffer_size_);
  pub_core_pose_state_ = nh.advertise<geometry_msgs::PoseStamped>("core_pose_state_out", m_sett.pub_cb_buffer_size_);
  pub_core_odom_state_ = nh.advertise<nav_msgs::Odometry>("core_odom_state_out", m_sett.pub_cb_buffer_size_);
  pub_gps1_state_ =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps1_cal_state_out", m_sett.pub_cb_buffer_size_);

  pub_gps1_enu_odom_ = nh.advertise<nav_msgs::Odometry>("gps1_enu", m_sett.pub_cb_buffer_size_);
}

bool MarsWrapperGpsVel::init()
{
  core_logic_.core_is_initialized_ = false;
  core_logic_.buffer_.ResetBufferData();
  gps1_sensor_sptr_->is_initialized_ = false;

  common_gps_ref_is_set_ = false;

  return true;
}

bool MarsWrapperGpsVel::initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                                            std_srvs::SetBool::Response& response)
{
  if (m_sett.bypass_init_service_)
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

void MarsWrapperGpsVel::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
{
  // Config parameter overwrite
  m_sett.publish_on_propagation_ = config.pub_on_prop;
  core_logic_.verbose_ = config.verbose;
  m_sett.verbose_output_ = config.verbose;
  m_sett.use_ros_time_now_ = config.use_ros_time_now;

  if (config.initialize)
  {
    init();
    ROS_INFO_STREAM("Initialized filter trough Reconfigure GUI");
  }

  config.initialize = false;
}

void MarsWrapperGpsVel::set_common_gps_reference(const GpsCoordinates& reference)
{
  if (!common_gps_ref_is_set_)
  {
    std::cout << "Setting the common GPS reference to: \n" << reference << std::endl;

    gps1_sensor_sptr_->set_gps_reference_coordinates(reference);

    common_gps_ref_is_set_ = true;
  }
}

void MarsWrapperGpsVel::ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas)
{
  // Map the measutement to the mars type

  Time timestamp;

  if (m_sett.use_ros_time_now_)
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
  if (!core_logic_.core_is_initialized_ && mag_init_.is_done())
  {
    core_logic_.Initialize(p_wi_init_, q_wi_init_);
    return;
  }

  if (m_sett.publish_on_propagation_)
  {
    RunCoreStatePublisher();
  }
}

void MarsWrapperGpsVel::Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& coord_meas,
                                                const geometry_msgs::TwistWithCovarianceStampedConstPtr& vel_meas)
{
  Time timestamp;

  if (m_sett.use_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(coord_meas->header.stamp.toSec());
  }

  // Map the coordinate and velocity measurements to the mars sensor type
  GpsVelMeasurementType gps_meas(MarsMsgConv::NavSatTwistMsgToGpsVelMeas(*coord_meas, *vel_meas));

  set_common_gps_reference(gps_meas.coordinates_);

  GpsVelMeasurementUpdate(gps1_sensor_sptr_, gps_meas, timestamp);

  // Publish GPS ENU as Odometry
  if (m_sett.publish_gps_enu_)
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

  mars::GpsVelSensorStateType gps_sensor_state = gps1_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);

  pub_gps1_state_.publish(
      MarsMsgConv::GpsVelStateToMsg(latest_sensor_state.timestamp_.get_seconds(), gps_sensor_state));
}

void MarsWrapperGpsVel::MagMeasurementCallback(const sensor_msgs::MagneticFieldConstPtr& meas)
{
  if (!mag_init_.is_done() && !this->m_sett.enable_manual_yaw_init_)
  {
    mag_init_.add_element(meas->magnetic_field.x, meas->magnetic_field.y, meas->magnetic_field.z);

    if (mag_init_.get_size() >= m_sett.auto_mag_init_samples_)
    {
      mag_init_.get_quat(q_wi_init_);
      std::cout << "Magnetometer yaw initialization: " << mag_init_.yaw_ * (180 / M_PI) << std::endl;

      mag_init_.set_done();
    }
  }
}

void MarsWrapperGpsVel::RunCoreStatePublisher()
{
  mars::BufferEntryType latest_state;
  const bool valid_state = core_logic_.buffer_.get_latest_state(&latest_state);

  if (!valid_state)
  {
    return;
  }

  mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

  pub_ext_core_state_.publish(MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
  pub_ext_core_state_lite_.publish(
      MarsMsgConv::ExtCoreStateLiteToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_pose_state_.publish(
      MarsMsgConv::ExtCoreStateToPoseMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  pub_core_odom_state_.publish(
      MarsMsgConv::ExtCoreStateToOdomMsg(latest_state.timestamp_.get_seconds(), latest_core_state));

  if (m_sett.cov_debug_)
  {
    CoreStateMatrix cov = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->cov_;

    double p_x = sqrt(cov(0, 0));
    double p_y = sqrt(cov(1, 1));
    double p_z = sqrt(cov(2, 2));

    double v_x = sqrt(cov(3, 3));
    double v_y = sqrt(cov(4, 4));
    double v_z = sqrt(cov(5, 5));

    double q_x = (180 / M_PI) * sqrt(cov(6, 6));
    double q_y = (180 / M_PI) * sqrt(cov(7, 7));
    double q_z = (180 / M_PI) * sqrt(cov(8, 8));

    double bw_x = (180 / M_PI) * sqrt(cov(9, 9));
    double bw_y = (180 / M_PI) * sqrt(cov(10, 10));
    double bw_z = (180 / M_PI) * sqrt(cov(11, 11));

    std::cout << "std p: x,y,z: " << p_x << " " << p_y << " " << p_z << std::endl;
    std::cout << "std y: x,y,z: " << v_x << " " << v_y << " " << v_z << std::endl;
    std::cout << "std q: x,y,z: " << q_x << " " << q_y << " " << q_z << std::endl;
    std::cout << "std b_w: x,y,z: " << bw_x << " " << bw_y << " " << bw_z << std::endl;
  }
}

void MarsWrapperGpsVel::GpsVelMeasurementUpdate(std::shared_ptr<mars::GpsVelSensorClass> sensor_sptr,
                                                const GpsVelMeasurementType& gps_meas, const Time& timestamp)
{
  // p_wi_init_ = pose_meas.position_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<GpsVelMeasurementType>(gps_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp, data))
  {
    return;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();
}
