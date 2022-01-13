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

#include "mars_wrapper_dualpose_full.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_measurement_type.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <mars_msg_conv.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>
#include <iostream>
#include <string>

#include "mars_msg_conv.h"

using namespace mars;

MarsWrapperDualPoseFull::MarsWrapperDualPoseFull(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&MarsWrapperDualPoseFull::configCallback, this, _1, _2))
  , m_sett(nh)
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
  m_sett.printAll();

  reconfigure_srv_.setCallback(reconfigure_cb_);

  initialization_service_ = nh.advertiseService("init_service", &MarsWrapperDualPoseFull::initServiceCallback, this);

  std::cout << "Setup framework components" << std::endl;

  // Framework components
  imu_sensor_sptr_ = std::make_shared<mars::ImuSensorClass>("IMU");
  core_states_sptr_ = std::make_shared<mars::CoreState>();
  core_states_sptr_.get()->set_initial_covariance(m_sett.core_init_cov_p_, m_sett.core_init_cov_v_,
                                                  m_sett.core_init_cov_q_, m_sett.core_init_cov_bw_,
                                                  m_sett.core_init_cov_ba_);

  core_states_sptr_.get()->set_propagation_sensor(imu_sensor_sptr_);
  core_logic_ = mars::CoreLogic(core_states_sptr_);
  core_logic_.buffer_.set_max_buffer_size(800);                  /// \todo TODO(scm): make this a param
  core_logic_.buffer_prior_core_init_.set_max_buffer_size(800);  /// \todo TODO(scm): make this a param

  core_logic_.verbose_ = m_sett.verbose_output_;
  core_logic_.verbose_out_of_order_ = m_sett.verbose_ooo_;
  core_logic_.discard_ooo_prop_meas_ = m_sett.discard_ooo_prop_meas_;

  core_states_sptr_->set_noise_std(Eigen::Vector3d(m_sett.g_rate_noise_, m_sett.g_rate_noise_, m_sett.g_rate_noise_),
                                   Eigen::Vector3d(m_sett.g_bias_noise_, m_sett.g_bias_noise_, m_sett.g_bias_noise_),
                                   Eigen::Vector3d(m_sett.a_noise_, m_sett.a_noise_, m_sett.a_noise_),
                                   Eigen::Vector3d(m_sett.a_bias_noise_, m_sett.a_bias_noise_, m_sett.a_bias_noise_));

  // Sensors
  // Pose1
  pose1_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose1", core_states_sptr_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett.pose1_pos_meas_noise_, m_sett.pose1_att_meas_noise_;
    pose1_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);

    PoseSensorData pose_calibration;
    pose_calibration.state_.p_ip_ = m_sett.pose1_cal_p_ip_;
    pose_calibration.state_.q_ip_ = m_sett.pose1_cal_q_ip_;
    Eigen::Matrix<double, 6, 6> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << m_sett.pose1_state_init_cov_;  // no calibration update
    pose_calibration.sensor_cov_ = pose_cov;
    pose1_sensor_sptr_->set_initial_calib(std::make_shared<PoseSensorData>(pose_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    pose1_sensor_sptr_->const_ref_to_nav_ = true;

    std::cout << "Info: [" << pose1_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << pose_calibration.state_.p_ip_.transpose() << " ]" << std::endl;
    std::cout << "\tOrientation[1]: [" << pose_calibration.state_.q_ip_.w() << " "
              << pose_calibration.state_.q_ip_.vec().transpose() << " ]" << std::endl;
    std::cout << "\tOrientation[deg]: ["
              << pose_calibration.state_.q_ip_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI)
              << " ]" << std::endl;
  }

  // Pose1
  pose2_sensor_sptr_ = std::make_shared<mars::PoseSensorClass>("Pose2", core_states_sptr_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 6, 1> pose_meas_std;
    pose_meas_std << m_sett.pose1_pos_meas_noise_, m_sett.pose2_att_meas_noise_;
    pose2_sensor_sptr_->R_ = pose_meas_std.cwiseProduct(pose_meas_std);

    PoseSensorData pose_calibration;
    pose_calibration.state_.p_ip_ = m_sett.pose2_cal_p_ip_;
    pose_calibration.state_.q_ip_ = m_sett.pose2_cal_q_ip_;
    Eigen::Matrix<double, 6, 6> pose_cov;
    pose_cov.setZero();
    pose_cov.diagonal() << m_sett.pose2_state_init_cov_;  // no calibration update
    pose_calibration.sensor_cov_ = pose_cov;
    pose2_sensor_sptr_->set_initial_calib(std::make_shared<PoseSensorData>(pose_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    pose2_sensor_sptr_->const_ref_to_nav_ = true;

    std::cout << "Info: [" << pose2_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << pose_calibration.state_.p_ip_.transpose() << " ]" << std::endl;
    std::cout << "\tOrientation[1]: [" << pose_calibration.state_.q_ip_.w() << " "
              << pose_calibration.state_.q_ip_.vec().transpose() << " ]" << std::endl;
    std::cout << "\tOrientation[deg]: ["
              << pose_calibration.state_.q_ip_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() * (180 / M_PI)
              << " ]" << std::endl;
  }

#ifndef GPS_W_VEL
  // GPS1
  gps1_sensor_sptr_ = std::make_shared<mars::GpsSensorClass>("Gps1", core_states_sptr_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 3, 1> gps_meas_std;
    gps_meas_std << m_sett.gps_pos_meas_noise_;
    gps1_sensor_sptr_->R_ = gps_meas_std.cwiseProduct(gps_meas_std);

    GpsSensorData gps_calibration;
    gps_calibration.state_.p_ig_ = Eigen::Vector3d(m_sett.gps_cal_ig_);
    Eigen::Matrix<double, 9, 9> gps_cov;
    gps_cov.setZero();
    gps_cov.diagonal() << m_sett.gps_state_init_cov_, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
    gps_calibration.sensor_cov_ = gps_cov;

    gps1_sensor_sptr_->set_initial_calib(std::make_shared<GpsSensorData>(gps_calibration));

    // TODO is set here for now, but will be managed by core logic in later versions
    gps1_sensor_sptr_->const_ref_to_nav_ = true;

    std::cout << "Info: [" << gps1_sensor_sptr_->name_ << "] Calibration(rounded):" << std::endl;
    std::cout << "\tPosition[m]: [" << gps_calibration.state_.p_ig_.transpose() << " ]" << std::endl;
  }
#else
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
#endif

  // Pressure sensor
  pressure_sensor_sptr_ = std::make_shared<mars::PressureSensorClass>("Pressure", core_states_sptr_);
  {  // Limit scope of temp variables
    Eigen::Matrix<double, 1, 1> pressure_meas_std;
    pressure_meas_std << m_sett.baro_meas_noise_;
    pressure_sensor_sptr_->R_ = pressure_meas_std.cwiseProduct(pressure_meas_std);

    PressureSensorData pressure_calibration;
    pressure_calibration.state_.p_ip_ = Eigen::Vector3d(m_sett.baro_cal_ip_);
    Eigen::Matrix<double, 3, 3> pressure_cov;
    pressure_cov.setZero();
    pressure_cov.diagonal() << m_sett.baro_state_init_cov_;
    pressure_calibration.sensor_cov_ = pressure_cov;
    pressure_sensor_sptr_->set_initial_calib(std::make_shared<PressureSensorData>(pressure_calibration));
    // TODO is set here for now, but will be managed by core logic in later versions
    pressure_sensor_sptr_->const_ref_to_nav_ = true;

    press_init_ = mars::PressureInit(m_sett.pressure_init_duration_);
  }

  // Subscriber
  sub_imu_measurement_ =
      nh.subscribe("imu_in", m_sett.sub_imu_cb_buffer_size_, &MarsWrapperDualPoseFull::ImuMeasurementCallback, this);

  sub_pose1_measurement_ = nh.subscribe("pose1_in", m_sett.sub_sensor_cb_buffer_size_,
                                        &MarsWrapperDualPoseFull::Pose1MeasurementCallback, this);
  sub_pose1_with_cov_measurement_ = nh.subscribe("pose1_with_cov_in", m_sett.sub_sensor_cb_buffer_size_,
                                                 &MarsWrapperDualPoseFull::Pose1WithCovMeasurementCallback, this);
  sub_odom1_measurement_ = nh.subscribe("odom1_in", m_sett.sub_sensor_cb_buffer_size_,
                                        &MarsWrapperDualPoseFull::Odom1MeasurementCallback, this);
  sub_transform1_measurement_ = nh.subscribe("transform1_in", m_sett.sub_sensor_cb_buffer_size_,
                                             &MarsWrapperDualPoseFull::Transform1MeasurementCallback, this);

  sub_pose2_measurement_ = nh.subscribe("pose2_in", m_sett.sub_sensor_cb_buffer_size_,
                                        &MarsWrapperDualPoseFull::Pose2MeasurementCallback, this);
  sub_pose2_with_cov_measurement_ = nh.subscribe("pose2_with_cov_in", m_sett.sub_sensor_cb_buffer_size_,
                                                 &MarsWrapperDualPoseFull::Pose2WithCovMeasurementCallback, this);
  sub_odom2_measurement_ = nh.subscribe("odom2_in", m_sett.sub_sensor_cb_buffer_size_,
                                        &MarsWrapperDualPoseFull::Odom2MeasurementCallback, this);
  sub_transform2_measurement_ = nh.subscribe("transform2_in", m_sett.sub_sensor_cb_buffer_size_,
                                             &MarsWrapperDualPoseFull::Transform2MeasurementCallback, this);

#ifndef GPS_W_VEL
  sub_gps1_measurement_ = nh.subscribe("gps1_in", m_sett.sub_sensor_cb_buffer_size_,
                                       &MarsWrapperDualPoseFull::Gps1MeasurementCallback, this);
#else
  sub_gps1_coord_meas_.subscribe(nh, "gps1_coord_in", m_sett.sub_sensor_cb_buffer_size_);
  sub_gps1_vel_meas_.subscribe(nh, "gps1_vel_in", m_sett.sub_sensor_cb_buffer_size_);
  sync_gps1_meas_.registerCallback(boost::bind(&MarsWrapperDualPoseFull::Gps1MeasurementCallback, this, _1, _2));
#endif

  if (m_sett.pressure_const_temp_)
  {
    sub_pressure_measurement_ = nh.subscribe("pressure_in", m_sett.sub_sensor_cb_buffer_size_,
                                             &MarsWrapperDualPoseFull::PressureMeasurementCallback, this);
  }
  else
  {
    // TODO
  }

  // Publisher
  pub_ext_core_state_ = nh.advertise<mars_ros::ExtCoreState>("core_ext_state_out", m_sett.pub_cb_buffer_size_);
  pub_ext_core_state_lite_ =
      nh.advertise<mars_ros::ExtCoreStateLite>("core_ext_state_lite_out", m_sett.pub_cb_buffer_size_);
  pub_core_pose_state_ = nh.advertise<geometry_msgs::PoseStamped>("core_pose_state_out", m_sett.pub_cb_buffer_size_);
  pub_core_odom_state_ = nh.advertise<nav_msgs::Odometry>("core_odom_state_out", m_sett.pub_cb_buffer_size_);
  pub_pose1_state_ = nh.advertise<geometry_msgs::PoseStamped>("pose1_cal_state_out", m_sett.pub_cb_buffer_size_);
  pub_pose2_state_ = nh.advertise<geometry_msgs::PoseStamped>("pose2_cal_state_out", m_sett.pub_cb_buffer_size_);
  pub_gps1_state_ =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps1_cal_state_out", m_sett.pub_cb_buffer_size_);
  pub_gps1_enu_odom_ = nh.advertise<nav_msgs::Odometry>("gps1_enu", m_sett.pub_cb_buffer_size_);
}

bool MarsWrapperDualPoseFull::init()
{
  core_logic_.core_is_initialized_ = false;
  core_logic_.buffer_.ResetBufferData();
  pose1_sensor_sptr_->is_initialized_ = false;
  pose2_sensor_sptr_->is_initialized_ = false;
  gps1_sensor_sptr_->is_initialized_ = false;
  pressure_sensor_sptr_->is_initialized_ = false;

  common_gps_ref_is_set_ = false;
  have_pose1_ = false;
  have_pose2_ = false;

  press_init_.Reset();

  return true;
}

bool MarsWrapperDualPoseFull::initServiceCallback(std_srvs::SetBool::Request& /*request*/,
                                                  std_srvs::SetBool::Response& res)
{
  init();
  res.success = true;
  ROS_INFO_STREAM("Initialized filter trough ROS Service");

  return true;
}

void MarsWrapperDualPoseFull::configCallback(mars_ros::marsConfig& config, uint32_t /*level*/)
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

void MarsWrapperDualPoseFull::set_common_gps_reference(const GpsCoordinates& reference, const Time& timestamp)
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

        if (m_sett.verbose_output_)
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

void MarsWrapperDualPoseFull::set_common_pressure_reference(const mars::Pressure& reference,
                                                            const mars::Time& timestamp)
{
  if (!press_init_.IsDone())
  {
    mars::Pressure mean =
        press_init_.get_press_mean(pressure_sensor_sptr_, core_logic_.buffer_prior_core_init_, reference, timestamp);

    if (press_init_.IsDone())
    {
      if (m_sett.verbose_output_)
        std::cout << "Setting the common pressure reference to: \n" << mean << std::endl;
      pressure_sensor_sptr_->set_pressure_reference(mean.data_, mean.temperature_K_);
    }
  }  // if (!common_pressure_ref_is_set_)
}

void MarsWrapperDualPoseFull::ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas)
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
  if (!core_logic_.core_is_initialized_ && have_pose1_ && press_init_.IsDone())
  {
    core_logic_.Initialize(p_wi_init_, q_wi_init_);
  }

  if (m_sett.publish_on_propagation_)
  {
    this->RunCoreStatePublisher();
  }
}

void MarsWrapperDualPoseFull::Pose1MeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::PoseMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Pose1WithCovMeasurementCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::PoseWithCovMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Transform1MeasurementCallback(const geometry_msgs::TransformStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::TransformMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Odom1MeasurementCallback(const nav_msgs::OdometryConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::OdomMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose1_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Pose2MeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::PoseMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Pose2WithCovMeasurementCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::PoseWithCovMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Transform2MeasurementCallback(const geometry_msgs::TransformStampedConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::TransformMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

void MarsWrapperDualPoseFull::Odom2MeasurementCallback(const nav_msgs::OdometryConstPtr& meas)
{
  // Map the measurement to the mars sensor type
  Time timestamp(meas->header.stamp.toSec());

  PoseMeasurementType pose_meas = MarsMsgConv::OdomMsgToPoseMeas(*meas);
  PoseMeasurementUpdate(pose2_sensor_sptr_, pose_meas, timestamp);
}

#ifndef GPS_W_VEL
void MarsWrapperDualPoseFull::Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas)
{
  Time timestamp;

  // use ros time for GPS regardless (due to uncertainty of timestamp from mavros GPS measurements)
  if (m_sett.gps_ros_time_now_)
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
    if (m_sett.publish_gps_enu_)
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
void MarsWrapperDualPoseFull::Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& coord_meas,
                                                      const geometry_msgs::TwistStampedConstPtr& vel_meas)
{
  Time timestamp;

  if (m_sett.gps_ros_time_now_)
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

void MarsWrapperDualPoseFull::PressureMeasurementCallback(const sensor_msgs::FluidPressureConstPtr& meas)
{
  Time timestamp;

  if (m_sett.use_ros_time_now_)
  {
    timestamp = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp = Time(meas->header.stamp.toSec());
  }

  PressureMeasurementType press_meas(MarsMsgConv::FluidPressureMsgtoPressureMeas(*meas, m_sett.pressure_temp_K_));
  set_common_pressure_reference(press_meas.pressure_, timestamp);

  // perform update
  if (PressureMeasurementUpdate(pressure_sensor_sptr_, press_meas, timestamp))
  {
    // Generate a measurement data block
    //    BufferDataType data;
    //    data.set_sensor_data(std::make_shared<PressureMeasurementType>(press_meas));

    //    mars::BufferEntryType latest_sensor_state;
    //    core_logic_.buffer_.get_latest_sensor_handle_state(pressure_sensor_sptr_, &latest_sensor_state);

    //    mars::PressureSensorStateType pressure_sensor_state =
    //        pressure_sensor_sptr_.get()->get_state(latest_sensor_state.data_.sensor_);
  }
}

void MarsWrapperDualPoseFull::RunCoreStatePublisher()
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

bool MarsWrapperDualPoseFull::PoseMeasurementUpdate(std::shared_ptr<mars::PoseSensorClass> sensor_sptr,
                                                    const PoseMeasurementType& pose_meas, const Time& timestamp)
{
  Time timestamp_corr;

  if (m_sett.use_ros_time_now_)
  {
    timestamp_corr = Time(ros::Time::now().toSec());
  }
  else
  {
    timestamp_corr = timestamp;
  }

  PoseMeasurementType rotated_pose_meas = pose_meas;

  // TMP feedback init pose (with sensor no1)
  if (sensor_sptr->name_ == "Pose1")
  {
    p_wi_init_ = pose_meas.position_;
    q_wi_init_ = pose_meas.orientation_;
    have_pose1_ = true;
  }
  else if (sensor_sptr->name_ == "Pose2")
  {
    /// \todo TODO(scm): this should probably be a pose_sensor conversion class!
    if (have_pose2_)
    {
      // measurement is in 'global' frame of pose 2 (gp2) to pose sensor
      // calibration is from vision (pose 1 global frame, and Mars global frame, i.e. w) to gp2
      // hence
      // p_wp = p_w_gp2 (here IP) + R_w_gp2 (here IP) * meas_gp2_p
      // q_wp = q_w_gp2 (here IP) * meas_gp2_p
      rotated_pose_meas.position_ =
          pose12_calibration_.state_.p_ip_ + pose12_calibration_.state_.q_ip_.toRotationMatrix() * pose_meas.position_;
      rotated_pose_meas.orientation_ = pose12_calibration_.state_.q_ip_ * pose_meas.orientation_;
    }
    else if (core_logic_.core_is_initialized_)
    {
      // get current core state
      mars::BufferEntryType latest_state;
      core_logic_.buffer_.get_latest_state(&latest_state);
      mars::CoreStateType latest_core_state = static_cast<mars::CoreType*>(latest_state.data_.core_.get())->state_;

      // save sensor 1 to sensor 2 calibration!
      /// \note NOTE(scm): variable names are wrong here, reusage of existing variables with their naming conventions,
      /// however it should be noted that _ip rather stands for _v_gp2, i.e. from the vision frame to the 'global' frame
      /// of pose sensor 2.
      Eigen::Matrix3d R_ip =
          latest_core_state.q_wi_.toRotationMatrix() * pose_meas.orientation_.toRotationMatrix().transpose();
      pose12_calibration_.state_.q_ip_ = Eigen::Quaterniond(R_ip);
      pose12_calibration_.state_.p_ip_ = latest_core_state.p_wi_ - R_ip * pose_meas.position_;
      Eigen::Matrix<double, 6, 6> pose_cov;

      ROS_INFO_STREAM("Pose2 initial calib: \n\tpos: " << pose12_calibration_.state_.p_ip_.transpose()
                                                       << "\n\tori: " << pose12_calibration_.state_.q_ip_.w() << " "
                                                       << pose12_calibration_.state_.q_ip_.vec().transpose());

      have_pose2_ = true;
    }
    else
      return false;
  }

  // Generate a measurement data block
  BufferDataType data;
  //  data.set_sensor_data(std::make_shared<PoseMeasurementType>(pose_meas));
  data.set_sensor_data(std::make_shared<PoseMeasurementType>(rotated_pose_meas));

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp_corr, data))
  {
    return false;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  // Publish the latest sensor state
  mars::BufferEntryType latest_result;
  core_logic_.buffer_.get_latest_sensor_handle_state(sensor_sptr, &latest_result);
  mars::PoseSensorStateType pose_sensor_state = sensor_sptr.get()->get_state(latest_result.data_.sensor_);

  pub_pose1_state_.publish(MarsMsgConv::PoseStateToPoseMsg(latest_result.timestamp_.get_seconds(), pose_sensor_state));

  return true;
}

#ifndef GPS_W_VEL
bool MarsWrapperDualPoseFull::GpsMeasurementUpdate(std::shared_ptr<mars::GpsSensorClass> sensor_sptr,
                                                   const GpsMeasurementType& gps_meas, const Time& timestamp)
{
  // TMP feedback init pose
  // p_wi_init_ = pose_meas.position_;
  // q_wi_init_ = pose_meas.orientation_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<GpsMeasurementType>(gps_meas));

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
#else
bool MarsWrapperDualPoseFull::GpsVelMeasurementUpdate(std::shared_ptr<mars::GpsVelSensorClass> sensor_sptr,
                                                      const GpsVelMeasurementType& gps_meas, const Time& timestamp)
{
  // p_wi_init_ = pose_meas.position_;

  // Generate a measurement data block
  BufferDataType data;
  data.set_sensor_data(std::make_shared<GpsVelMeasurementType>(gps_meas));

  // Update init buffer
  BufferEntryType gps_meas_entry(timestamp, data, gps1_sensor_sptr_, BufferMetadataType::measurement);
  gps_meas_buffer_.AddEntrySorted(gps_meas_entry);

  // Call process measurement
  if (!core_logic_.ProcessMeasurement(sensor_sptr, timestamp, data))
  {
    return false;
  }

  // Publish the latest core state
  this->RunCoreStatePublisher();

  return true;
}
#endif

bool MarsWrapperDualPoseFull::PressureMeasurementUpdate(std::shared_ptr<mars::PressureSensorClass> sensor_sptr,
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
