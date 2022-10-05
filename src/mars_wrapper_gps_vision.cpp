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

#include "mars_wrapper_gps_vision.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/vision/vision_measurement_type.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars_ros/ExtCoreState.h>
#include <mars_ros/ExtCoreStateLite.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>

using namespace mars;

MarsWrapperGpsVision::MarsWrapperGpsVision(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperGpsVision::configCallback, this, _1, _2))
  , m_sett_(nh)
#ifdef GPS_W_VEL
#if NOT APPROX_TIME_SYNC
  , sync_gps1_meas_(sub_gps1_coord_meas_, sub_gps1_vel_meas_, sub_sensor_cb_buffer_size_)
#else
  , sync_gps1_meas_(ApproxTimePolicy(3), sub_gps1_coord_meas_, sub_gps1_vel_meas_)
#endif
#endif  // GPS_W_VEL
  , p_wi_init_(0, 0, 0)
  , q_wi_init_(Eigen::Quaterniond::Identity())
{
  m_sett_.printAll();

  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperGpsVision::initServiceCallback, this);

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
  core_logic_.buffer_prior_core_init_.set_max_buffer_size(m_sett_.buffer_size_);

  core_logic_.verbose_ = m_sett_.verbose_output_;
  core_logic_.verbose_out_of_order_ = m_sett_.verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = m_sett_.discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(
      Eigen::Vector3d(m_sett_.g_rate_noise_, m_sett_.g_rate_noise_, m_sett_.g_rate_noise_),
      Eigen::Vector3d(m_sett_.g_bias_noise_, m_sett_.g_bias_noise_, m_sett_.g_bias_noise_),
      Eigen::Vector3d(m_sett_.a_noise_, m_sett_.a_noise_, m_sett_.a_noise_),
      Eigen::Vector3d(m_sett_.a_bias_noise_, m_sett_.a_bias_noise_, m_sett_.a_bias_noise_));

  // Sensors
  // Vision1
  vision1_sensor_sptr_ =
      std::make_shared<mars::VisionSensorClass>("Vision1", core_states_sptr_, !m_sett_.vision1_fixed_scale_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> vision_meas_std;
    vision_meas_std << m_sett_.vision1_pos_meas_noise_, m_sett_.vision1_att_meas_noise_;
    vision1_sensor_sptr_->R_ = vision_meas_std.cwiseProduct(vision_meas_std);
    vision1_sensor_sptr_->use_dynamic_meas_noise_ = false;

    // TODO is set here for now, but will be managed by core logic in later versions
    vision1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // MAG1
  mag1_sensor_sptr_ = std::make_shared<mars::MagSensorClass>("Mag1", core_states_sptr_);
  {
    // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> mag_meas_std;
    mag_meas_std << m_sett_.mag1_meas_noise_;
    mag1_sensor_sptr_->R_ = mag_meas_std.cwiseProduct(mag_meas_std);

    MagSensorData mag_calibration;
    mag_calibration.state_.mag_ = Eigen::Vector3d(0, 1, 0);

    mag_calibration.state_.q_im_ = Eigen::Quaterniond::Identity();
    Eigen::Matrix<double, 6, 6> mag_cov;
    mag_cov.setZero();
    mag_cov.diagonal() << m_sett_.mag1_state_init_cov_;
    mag_calibration.sensor_cov_ = mag_cov;

    mag1_sensor_sptr_->set_initial_calib(std::make_shared<MagSensorData>(mag_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    mag1_sensor_sptr_->const_ref_to_nav_ = true;
  }

  // GPS1
#ifndef GPS_W_VEL
  gps1_sensor_sptr_ = std::make_shared<mars::GpsSensorClass>("Gps1", core_states_sptr_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> gps1_meas_std;
    gps1_meas_std << m_sett_.gps1_pos_meas_noise_;
#else
  gps1_sensor_sptr_ = std::make_shared<mars::GpsVelSensorClass>("Gps1", core_states_sptr_);

  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> gps1_meas_std;

    gps1_meas_std << m_sett_.gps1_pos_meas_noise_, m_sett_.gps1_vel_meas_noise_;
#endif  // GPS_W_VEL
    gps1_sensor_sptr_->R_ = gps1_meas_std.cwiseProduct(gps1_meas_std);

#ifndef GPS_W_VEL
    GpsSensorData gps_calibration;
#else
    GpsVelSensorData gps_calibration;
#endif
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett_.gps1_cal_ig_);
    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett_.gps1_state_init_cov_, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gps_calibration.sensor_cov_ = gps_cov;

#ifndef GPS_W_VEL
    gps1_sensor_sptr_->set_initial_calib(std::make_shared<GpsSensorData>(gps_calibration));
#else
    gps1_sensor_sptr_->set_initial_calib(std::make_shared<GpsVelSensorData>(gps_calibration));
#endif

    // TODO is set here for now, but will be managed by core logic in later versions
    gps1_sensor_sptr_->const_ref_to_nav_ = true;

    std::cout << "Info: [" << gps1_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << gps_calibration.state_.p_ig_.transpose() << " ]" << std::endl;
  }

  // Pressure sensor
  pressure1_sensor_sptr_ = std::make_shared<mars::PressureSensorClass>("Pressure", core_states_sptr_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 1, 1> pressure_meas_std;
    pressure_meas_std << m_sett_.pressure1_meas_noise_;
    pressure1_sensor_sptr_->R_ = pressure_meas_std.cwiseProduct(pressure_meas_std);

    PressureSensorData pressure_calibration;
    pressure_calibration.state_.p_ip_ = Eigen::Vector3d(m_sett_.pressure1_cal_ip_);
    Eigen::Matrix<double, 3, 3> pressure_cov;
    pressure_cov.setZero();
    pressure_cov.diagonal() << m_sett_.pressure1_state_init_cov_;
    pressure_calibration.sensor_cov_ = pressure_cov;
    pressure1_sensor_sptr_->set_initial_calib(std::make_shared<PressureSensorData>(pressure_calibration));
    // TODO is set here for now, but will be managed by core logic in later versions
    pressure1_sensor_sptr_->const_ref_to_nav_ = true;

    press_init_ = mars::PressureInit(m_sett_.pressure1_init_duration_);
  }

  // Subscriber
  sub_imu_measurement_ =
      nh.subscribe("imu_in", m_sett_.sub_imu_cb_buffer_size_, &MarsWrapperGpsVision::ImuMeasurementCallback, this);

  sub_vision1_measurement_ = nh.subscribe("vision1_in", m_sett_.sub_sensor_cb_buffer_size_,
                                          &MarsWrapperGpsVision::Vision1MeasurementCallback, this);

#ifndef GPS_W_VEL
  sub_gps1_measurement_ =
      nh.subscribe("gps1_in", m_sett_.sub_sensor_cb_buffer_size_, &MarsWrapperGpsVision::Gps1MeasurementCallback, this);
#else
  sub_gps1_coord_meas_.subscribe(nh, "gps1_coord_in", m_sett_.sub_sensor_cb_buffer_size_);
  sub_gps1_vel_meas_.subscribe(nh, "gps1_vel_in", m_sett_.sub_sensor_cb_buffer_size_);
  sync_gps1_meas_.registerCallback(boost::bind(&MarsWrapperGpsVision::Gps1MeasurementCallback, this, _1, _2));
#endif

  if (m_sett_.use_pressure_ && m_sett_.pressure1_const_temp_)
  {
    sub_pressure1_measurement_ = nh.subscribe("pressure1_in", m_sett_.sub_sensor_cb_buffer_size_,
                                              &MarsWrapperGpsVision::Pressure1MeasurementCallback, this);
  }
  else
  {
    // TODO
  }

  if (m_sett_.use_magnetometer_)
  {
    sub_mag1_measurement_ = nh.subscribe("mag1_in", m_sett_.sub_sensor_cb_buffer_size_,
                                         &MarsWrapperGpsVision::Mag1MeasurementCallback, this);
  }

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

  pub_vision1_state_ = nh.advertise<mars_ros::VisionSensorState>("vision1_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_gps1_state_ =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps1_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_mag1_state_ =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("mag1_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_pressure1_state_ =
      nh.advertise<geometry_msgs::PoseStamped>("pressure1_cal_state_out", m_sett_.pub_cb_buffer_size_);
  pub_gps1_enu_odom_ = nh.advertise<nav_msgs::Odometry>("gps1_enu", m_sett_.pub_cb_buffer_size_);
  pub_pressure1_height_vec3_ =
      nh.advertise<geometry_msgs::Vector3Stamped>("pressure1_height", m_sett_.pub_cb_buffer_size_);
}

bool MarsWrapperGpsVision::init()
{
  core_logic_.core_is_initialized_ = false;
  core_logic_.buffer_.ResetBufferData();
  vision1_sensor_sptr_->is_initialized_ = false;
  gps1_sensor_sptr_->is_initialized_ = false;
  pressure1_sensor_sptr_->is_initialized_ = false;
  mag1_sensor_sptr_->is_initialized_ = false;

  common_gps_ref_is_set_ = false;
  have_pose1_ = false;

  press_init_.Reset();
  mag_init_.Reset();

  return true;
}

bool MarsWrapperGpsVision::initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                                               std_srvs::SetBool::Response& res)
{
  init();
  res.success = true;
  ROS_INFO_STREAM("Initialized filter trough ROS Service");

  return true;
}

void MarsWrapperGpsVision::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
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

void MarsWrapperGpsVision::set_common_gps_reference(const GpsCoordinates& reference, const Time& timestamp)
{
  if (!common_gps_ref_is_set_)
  {
    // get oldest measurement in buffer
    BufferEntryType oldest_gps_meas;
    if (gps_meas_buffer_.get_entry_at_idx(0, &oldest_gps_meas))
    {
      // check if at least 1s has passed since first and current measurement
      if ((timestamp - oldest_gps_meas.timestamp_).get_seconds() > 1.0)
      {
        // iterate through buffer
        GpsCoordinates avg_ref = reference;
        BufferEntryType tmp_meas;
        uint cnt_avgs = 0;
        for (int i = 0; i < gps_meas_buffer_.get_length(); ++i)
        {
          // retrieve element and check if at the measurement happened within the last second
          gps_meas_buffer_.get_entry_at_idx(i, &tmp_meas);
          if ((timestamp - tmp_meas.timestamp_).get_seconds() < 1.0)
          {
            // add measruement to average reference
            GpsCoordinates tmp_coords = static_cast<GpsMeasurementType*>(tmp_meas.data_.sensor_.get())->coordinates_;
            avg_ref.altitude_ += tmp_coords.altitude_;
            avg_ref.latitude_ += tmp_coords.latitude_;
            avg_ref.longitude_ += tmp_coords.longitude_;

            // update counter
            cnt_avgs++;
          }  // if ((timestamp - tmp_meas.timestamp_).get_seconds() < 1.0)
        }    // for (int i = 0; i < gps_meas_buffer_.get_length(); ++i)
        avg_ref.altitude_ /= (cnt_avgs + 1);
        avg_ref.latitude_ /= (cnt_avgs + 1);
        avg_ref.longitude_ /= (cnt_avgs + 1);

        if (m_sett_.verbose_output_)
          std::cout << "Setting the common average GPS reference to: \n" << avg_ref << std::endl;

        gps1_sensor_sptr_->set_gps_reference_coordinates(avg_ref);

        common_gps_ref_is_set_ = true;
      }  // if ((timestamp - oldest_gps_meas.timestamp_).get_seconds() > 1.0)
      else
        ROS_INFO_STREAM("Not enough GPS measurements yet ...");
    }  // if (gps_meas_buffer_.get_entry_at_idx(0, &oldest_gps_meas))
    else
      ROS_INFO_STREAM("No received GPS measurement yet ...");
  }  // if (!common_gps_ref_is_set_)
}

void MarsWrapperGpsVision::set_common_pressure_reference(const mars::Pressure& reference, const mars::Time& timestamp)
{
  if (!press_init_.IsDone())
  {
    mars::Pressure mean =
        press_init_.get_press_mean(pressure1_sensor_sptr_, core_logic_.buffer_prior_core_init_, reference, timestamp);

    if (press_init_.IsDone())
    {
      if (m_sett_.verbose_output_)
        std::cout << "Setting the common pressure reference to: \n" << mean << std::endl;
      pressure1_sensor_sptr_->set_pressure_reference(mean.data_, mean.temperature_K_);
    }
  }  // if (!common_pressure_ref_is_set_)
}

void MarsWrapperGpsVision::ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas)
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
  core_logic_.ProcessMeasurement(imu_sensor_sptr_, timestamp, data);

  // Initialize the first time at which the propagation sensor occures
  if (!core_logic_.core_is_initialized_)
  {
    if (common_gps_ref_is_set_ && (press_init_.IsDone() || !m_sett_.use_pressure_) &&
        (mag_init_.IsDone() || !m_sett_.use_magnetometer_))
      core_logic_.Initialize(p_wi_init_, q_wi_init_);
  }
  else
  {
    if (m_sett_.publish_on_propagation_ && ++pub_prob_cnt % m_sett_.pub_prop_divider_ == 0)
    {
      this->RunCoreStatePublisher();
      pub_prob_cnt = 0;
    }
  }
}

void MarsWrapperGpsVision::Vision1MeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());
  VisionMeasurementType vision_meas = MarsMsgConv::PoseMsgToVisionMeas(*meas);

  if (VisionMeasurementUpdate(vision1_sensor_sptr_, vision_meas, timestamp))
  {
    // Publish the latest sensor state
    mars::BufferEntryType latest_result;
    if (core_logic_.buffer_.get_latest_sensor_handle_state(vision1_sensor_sptr_, &latest_result))
    {
      mars::VisionSensorStateType vision_sensor_state =
          vision1_sensor_sptr_.get()->get_state(latest_result.data_.sensor_);

      pub_vision1_state_.publish(
          MarsMsgConv::VisionStateToMsg(latest_result.timestamp_.get_seconds(), vision_sensor_state));
    }
  }
}

#ifndef GPS_W_VEL
void MarsWrapperGpsVision::Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas)
{
  Time timestamp;

  // use ros time for GPS regardless (due to uncertainty of timestamp from mavros GPS measurements)
  if (m_sett_.gps_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(meas->header.stamp.toSec());
  }

  // Map the measurement to the mars sensor type
  GpsMeasurementType gps_meas = MarsMsgConv::NavSatFixMsgToGpsMeas(*meas);
  set_common_gps_reference(gps_meas.coordinates_, timestamp);

  // Publish latest sensor state
  if (GpsMeasurementUpdate(gps1_sensor_sptr_, gps_meas, timestamp))
  {
    // Publish GPS ENU as Odometry
    if (m_sett_.publish_gps_enu_)
    {
      Eigen::Vector3d gps_enu(gps1_sensor_sptr_->gps_conversion_.get_enu(gps_meas.coordinates_));
      pub_gps1_enu_odom_.publish(MarsMsgConv::EigenVec3dToOdomMsg(timestamp.get_seconds(), gps_enu));
    }

    mars::BufferEntryType latest_sensor_state;
    core_logic_.buffer_.get_latest_sensor_handle_state(gps1_sensor_sptr_, &latest_sensor_state);

    mars::GpsSensorStateType gps_sensor_state = gps1_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);
    pub_gps1_state_.publish(MarsMsgConv::GpsStateToMsg(latest_sensor_state.timestamp_.get_seconds(), gps_sensor_state));
  }
}
#else
void MarsWrapperGpsVision::Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& coord_meas,
                                                   const geometry_msgs::TwistStampedConstPtr& vel_meas)
{
  Time timestamp;

  if (m_sett_.gps_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(coord_meas->header.stamp.toSec());
  }

  // Map the coordinate and velocity measurements to the mars sensor type
  GpsVelMeasurementType gps_meas(MarsMsgConv::NavSatTwistMsgToGpsVelMeas(*coord_meas, *vel_meas));

  set_common_gps_reference(gps_meas.coordinates_, timestamp);

  if (GpsVelMeasurementUpdate(gps1_sensor_sptr_, gps_meas, timestamp))
  {
    // Publish latest sensor state
    mars::BufferEntryType latest_sensor_state;
    const bool valid_state =
        core_logic_.buffer_.get_latest_sensor_handle_state(gps1_sensor_sptr_, &latest_sensor_state);

    if (!valid_state)
    {
      return;
    }

    mars::GpsVelSensorStateType gps_sensor_state =
        gps1_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);

    pub_gps1_state_.publish(
        MarsMsgConv::GpsVelStateToMsg(latest_sensor_state.timestamp_.get_seconds(), gps_sensor_state));
  }
}
#endif

void MarsWrapperGpsVision::Pressure1MeasurementCallback(const sensor_msgs::FluidPressureConstPtr& meas)
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

  PressureMeasurementType press_meas(MarsMsgConv::FluidPressureMsgtoPressureMeas(*meas, m_sett_.pressure1_temp_K_));
  set_common_pressure_reference(press_meas.pressure_, timestamp);

  // perform update
  if (PressureMeasurementUpdate(pressure1_sensor_sptr_, press_meas, timestamp))
  {
    // Publish latest sensor state
    mars::BufferEntryType latest_sensor_state;

    if (core_logic_.buffer_.get_latest_sensor_handle_state(pressure1_sensor_sptr_, &latest_sensor_state))
    {
      mars::PressureSensorStateType pressure_sensor_state =
          pressure1_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);
      pub_pressure1_state_.publish(
          MarsMsgConv::PressureStateToMsg(latest_sensor_state.timestamp_.get_seconds(), pressure_sensor_state));
    }

    // Publish pressure as vector
    if (m_sett_.publish_baro_height_)
    {
      Eigen::Vector3d baro_height;
      baro_height << 0, 0, pressure1_sensor_sptr_->pressure_conversion_.get_height(press_meas.pressure_);
      pub_pressure1_height_vec3_.publish(MarsMsgConv::EigenVec3dToVec3Msg(timestamp.get_seconds(), baro_height));
    }
  }
}

void MarsWrapperGpsVision::Mag1MeasurementCallback(const sensor_msgs::MagneticFieldConstPtr& meas)
{
  // Yaw initialization
  if (!mag_init_.IsDone() && !this->m_sett_.enable_manual_yaw_init_)
  {
    // Get last IMU measurement
    mars::BufferEntryType latest_imu_meas_entry;

    const bool found_imu_meas = core_logic_.buffer_prior_core_init_.get_latest_sensor_handle_measurement(
        imu_sensor_sptr_, &latest_imu_meas_entry);

    if (!found_imu_meas)
    {
      // Stop init routine if no previous IMU measurement was found
      return;
    };

    mars::IMUMeasurementType imu_meas =
        *static_cast<mars::IMUMeasurementType*>(latest_imu_meas_entry.data_.sensor_.get());

    // Get current Magnetometer measurement
    mars::MagMeasurementType mag_meas = MarsMsgConv::MagMsgToMagMeas(*meas);

    // Feed measurements to rotation init buffer
    mag_init_.AddElement(m_sett_.mag1_cal_q_im_.toRotationMatrix() * mag_meas.mag_vector_,
                         imu_meas.linear_acceleration_);

    if (mag_init_.get_size() >= m_sett_.auto_mag_init_samples_)
    {
      q_wi_init_ = mag_init_.get_quat();
      mag_init_.set_done();
    }

    return;
  }

  Time timestamp;

  if (m_sett_.use_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(meas->header.stamp.toSec());
  }

  // Map the magnetometer measurement to the mars sensor type
  MagMeasurementType mag_meas(MarsMsgConv::MagMsgToMagMeas(*meas));

  if (m_sett_.mag1_normalize_)
  {
    mag_meas.mag_vector_ = mag_meas.mag_vector_ / mag_meas.mag_vector_.norm();
  }

  if (MagMeasurementUpdate(mag1_sensor_sptr_, mag_meas, timestamp))
  {
    // Publish latest sensor state
    mars::BufferEntryType latest_sensor_state;

    if (core_logic_.buffer_.get_latest_sensor_handle_state(mag1_sensor_sptr_, &latest_sensor_state))
    {
      mars::MagSensorStateType mag_sensor_state = mag1_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);
      pub_mag1_state_.publish(
          MarsMsgConv::MagStateToMsg(latest_sensor_state.timestamp_.get_seconds(), mag_sensor_state));
    }
  }
}

void MarsWrapperGpsVision::RunCoreStatePublisher()
{
  mars::BufferEntryType latest_state;

  // only publish valid states
  if (core_logic_.buffer_.get_latest_state(&latest_state))
  {
    mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

    pub_ext_core_state_.publish(
        MarsMsgConv::ExtCoreStateToMsg(latest_state.timestamp_.get_seconds(), latest_core_state));
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
}

bool MarsWrapperGpsVision::VisionMeasurementUpdate(std::shared_ptr<mars::VisionSensorClass> sensor_sptr,
                                                   const mars::VisionMeasurementType& vision_meas,
                                                   const mars::Time& timestamp)
{
  Time timestamp_corr;

  if (m_sett_.use_ros_time_now_)
  {
    timestamp_corr = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp_corr = timestamp;
  }

  // initialize vision sensor using world position from GPS
  if (!have_pose1_)
  {
    // check if core is initialized
    if (!core_logic_.core_is_initialized_)
      return false;

    // get latest core
    mars::BufferEntryType latest_state;
    core_logic_.buffer_.get_latest_state(&latest_state);
    mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

    // first measurement, initialize initial calib here
    VisionSensorData vision_calibration;

    Eigen::Vector3d p_vc(vision_meas.position_);
    Eigen::Quaterniond q_vc(vision_meas.orientation_);

    Eigen::Vector3d p_wi(latest_core_state.p_wi_);
    Eigen::Quaterniond q_wi(latest_core_state.q_wi_);
    Eigen::Matrix3d r_wi(q_wi.toRotationMatrix());

    // set lamda to 1
    vision_calibration.state_.lambda_ = 1;

    // cam imu calibration from params
    vision_calibration.state_.p_ic_ = m_sett_.vision1_cal_p_ip_;
    vision_calibration.state_.q_ic_ = m_sett_.vision1_cal_q_ip_;

    // calculate initial vision to world transform
    Eigen::Quaterniond q_vw_ = q_vc * m_sett_.vision1_cal_q_ip_.conjugate() * q_wi.conjugate();
    vision_calibration.state_.q_vw_ = q_vw_;
    vision_calibration.state_.p_vw_ = p_vc - q_vw_.toRotationMatrix() * (p_wi + r_wi * m_sett_.vision1_cal_p_ip_);

    Eigen::Matrix<double, 13, 13> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, m_sett_.vision1_cal_ip_init_cov_,
        m_sett_.vision1_scale_init_cov_;  // no calibration update
    vision_calibration.sensor_cov_ = pose_cov;
    vision1_sensor_sptr_->set_initial_calib(std::make_shared<VisionSensorData>(vision_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    vision1_sensor_sptr_->const_ref_to_nav_ = true;

    std::cout << "Info: [" << vision1_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << vision_calibration.state_.p_ic_.transpose() << " ]" << std::endl;
    std::cout << "\tOrientation[1]: [" << vision_calibration.state_.q_ic_.w() << " "
              << vision_calibration.state_.q_ic_.vec().transpose() << " ]" << std::endl;
    std::cout << "\tOrientation[deg]: ["
              << vision_calibration.state_.q_ic_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI)
              << " ]" << std::endl;
    std::cout << "\tWorldOffset[m]: [" << vision_calibration.state_.p_vw_.transpose() << " ]" << std::endl;
    std::cout << "\tWorldOffset[1]: [" << vision_calibration.state_.q_vw_.w() << " "
              << vision_calibration.state_.q_vw_.vec().transpose() << " ]" << std::endl;
    std::cout << "\tWorldOffset[deg]: ["
              << vision_calibration.state_.q_vw_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI)
              << " ]" << std::endl;
    std::cout << "\tScale[deg]: [" << vision_calibration.state_.lambda_ << " ]" << std::endl;

    have_pose1_ = true;
  }

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<VisionMeasurementType>(vision_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp_corr, data))
  {
    return false;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  return true;
}

#ifndef GPS_W_VEL
bool MarsWrapperGpsVision::GpsMeasurementUpdate(std::shared_ptr<mars::GpsSensorClass> sensor_sptr,
                                                const GpsMeasurementType& gps_meas, const Time& timestamp)
{
  // TMP feedback init pose
  // p_wi_init_ = pose_meas.position_;
  // q_wi_init_ = pose_meas.orientation_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<GpsMeasurementType>(gps_meas));
#else
bool MarsWrapperGpsVision::GpsVelMeasurementUpdate(std::shared_ptr<mars::GpsVelSensorClass> sensor_sptr,
                                                   const GpsVelMeasurementType& gps_meas, const Time& timestamp)
{
  // p_wi_init_ = pose_meas.position_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<GpsVelMeasurementType>(gps_meas));
#endif

  // Update init buffer
  BufferEntryType gps_meas_entry(timestamp, data, gps1_sensor_sptr_, BufferMetadataType::measurement);
  gps_meas_buffer_.AddEntrySorted(gps_meas_entry);

  // only continue if we have common gps ref set
  if (common_gps_ref_is_set_)
  {
    // Call process measurement
    if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp, data))
    {
      return false;
    }

    // Publish the latest core state
    this->RunCoreStatePublisher();
  }

  return common_gps_ref_is_set_;
}

bool MarsWrapperGpsVision::PressureMeasurementUpdate(std::shared_ptr<mars::PressureSensorClass> sensor_sptr,
                                                     const mars::PressureMeasurementType& press_meas,
                                                     const mars::Time& timestamp)
{
  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<PressureMeasurementType>(press_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp, data))
  {
    return false;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  return true;
}

bool MarsWrapperGpsVision::MagMeasurementUpdate(std::shared_ptr<MagSensorClass> sensor_sptr,
                                                const MagMeasurementType& mag_meas, const Time& timestamp)
{
  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<MagMeasurementType>(mag_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp, data))
  {
    return false;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  return true;
}
