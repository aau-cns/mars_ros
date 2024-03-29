// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARS_WRAPPER_GPS_MAG_H
#define MARS_WRAPPER_GPS_MAG_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_sensor_class.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/mag/mag_measurement_type.h>
#include <mars/sensors/mag/mag_sensor_class.h>
#include <mars/sensors/mag/mag_utils.h>
#include <mars_msg_conv.h>
#include <mars_ros/marsConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>

#include <boost/bind/bind.hpp>

class ParamLoad
{
public:
  bool publish_on_propagation_{ true };  ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };       ///< Set to true to use rostime now for all sensor updates
  bool verbose_output_{ false };         ///< If true, all verbose infos are printed
  bool verbose_ooo_{ true };             ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };  ///< If true, all out of order propagation sensor meas are discarded
  bool pub_cov_{ true };                 ///< Publish covariances in the ext core state message if true
  bool pub_path_{ false };               ///< Publish all core states as nav_msgs::Path (for rviz)
  uint32_t buffer_size_{ 2000 };         ///< Set mars buffersize

  bool use_tcpnodelay_{ true };  ///< Use tcp no delay for the ROS msg. system
  bool bypass_init_service_{ false };

  uint32_t pub_cb_buffer_size_{ 1 };         ///< Callback buffersize for all outgoing topics
  uint32_t sub_imu_cb_buffer_size_{ 200 };   ///< Callback buffersize for propagation sensor measurements
  uint32_t sub_sensor_cb_buffer_size_{ 1 };  ///< Callback buffersize for all non-propagation sensor measurements

  bool use_common_gps_reference_{ true };  ///< Use a common GPS reference for all sensors
  bool publish_gps_enu_{ false };          ///< Publish GPS as ENU in the ref. frame used by the filter
  bool enable_manual_yaw_init_{ false };   ///< Initialize the yaw based on 'yaw_init_deg_'
  double yaw_init_deg_{ 0 };               ///< Yaw for core state init if 'enable_manual_yaw_init_' is true
  int auto_mag_init_samples_{ 30 };        ///< Number if meas. sample if auto init is used
                                           ///(enable_manual_yaw_init_=false)

  double g_rate_noise_;
  double g_bias_noise_;
  double a_noise_;
  double a_bias_noise_;

  Eigen::Vector3d core_init_cov_p_;
  Eigen::Vector3d core_init_cov_v_;
  Eigen::Vector3d core_init_cov_q_;
  Eigen::Vector3d core_init_cov_bw_;
  Eigen::Vector3d core_init_cov_ba_;

  Eigen::Vector3d gps1_pos_meas_noise_;
  Eigen::Vector3d gps1_vel_meas_noise_;
  bool gps1_use_dyn_meas_noise_{ false };
  Eigen::Vector3d gps1_cal_ig_;
  Eigen::Vector3d gps1_state_init_cov_;

  bool mag1_normalize_{ true };
  double mag1_declination_{ 0.0 };
  Eigen::Vector3d mag1_meas_noise_;
  bool mag1_use_dyn_meas_noise_{ false };
  Eigen::Quaterniond mag1_cal_q_im_;
  Eigen::Matrix<double, 6, 1> mag1_state_init_cov_;

  void check_size(const int& size_in, const int& size_comp)
  {
    if (size_comp != size_in)
    {
      std::cerr << "YAML array with wrong size" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  ParamLoad(const ros::NodeHandle& nh)
  {
    // Framework settings
    publish_on_propagation_ = nh.param<bool>("pub_on_prop", publish_on_propagation_);
    use_ros_time_now_ = nh.param<bool>("use_ros_time_now", use_ros_time_now_);
    verbose_output_ = nh.param<bool>("verbose", verbose_output_);
    verbose_ooo_ = nh.param<bool>("verbose_out_of_order", verbose_ooo_);
    discard_ooo_prop_meas_ = nh.param<bool>("discard_ooo_prop_meas", discard_ooo_prop_meas_);
    pub_cov_ = nh.param<bool>("pub_cov", pub_cov_);
    pub_path_ = nh.param<bool>("pub_path", pub_path_);
    buffer_size_ = nh.param<int>("buffer_size", buffer_size_);

    // ROS Settings
    use_tcpnodelay_ = nh.param<bool>("use_tcpnodelay", use_tcpnodelay_);
    bypass_init_service_ = nh.param<bool>("bypass_init_service", bypass_init_service_);

    pub_cb_buffer_size_ = uint32_t(nh.param<int>("pub_cb_buffer_size", int(pub_cb_buffer_size_)));
    sub_imu_cb_buffer_size_ = uint32_t(nh.param<int>("sub_imu_cb_buffer_size", int(sub_imu_cb_buffer_size_)));
    sub_sensor_cb_buffer_size_ = uint32_t(nh.param<int>("sub_sensor_cb_buffer_size", int(sub_sensor_cb_buffer_size_)));

    // Node specific settings
    publish_gps_enu_ = nh.param<bool>("publish_gps_enu", publish_gps_enu_);
    use_common_gps_reference_ = nh.param<bool>("use_common_gps_reference", use_common_gps_reference_);

    // Yaw initialization
    enable_manual_yaw_init_ = nh.param<bool>("enable_manual_yaw_init", enable_manual_yaw_init_);
    nh.param("yaw_init_deg", yaw_init_deg_, double());
    auto_mag_init_samples_ = uint32_t(nh.param<int>("auto_mag_init_samples", int(auto_mag_init_samples_)));

    // IMU parameter
    nh.param("gyro_rate_noise", g_rate_noise_, double());
    nh.param("gyro_bias_noise", g_bias_noise_, double());
    nh.param("acc_noise", a_noise_, double());
    nh.param("acc_bias_noise", a_bias_noise_, double());

    // Core state covariance
    std::vector<double> core_init_cov_p;
    nh.param("core_init_cov_p", core_init_cov_p, std::vector<double>());
    check_size(core_init_cov_p.size(), 3);
    core_init_cov_p_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_p.data());

    std::vector<double> core_init_cov_v;
    nh.param("core_init_cov_v", core_init_cov_v, std::vector<double>());
    check_size(core_init_cov_v.size(), 3);
    core_init_cov_v_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_v.data());

    std::vector<double> core_init_cov_q;
    nh.param("core_init_cov_q", core_init_cov_q, std::vector<double>());
    check_size(core_init_cov_q.size(), 3);
    core_init_cov_q_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_q.data());

    std::vector<double> core_init_cov_bw;
    nh.param("core_init_cov_bw", core_init_cov_bw, std::vector<double>());
    check_size(core_init_cov_bw.size(), 3);
    core_init_cov_bw_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_bw.data());

    std::vector<double> core_init_cov_ba;
    nh.param("core_init_cov_ba", core_init_cov_ba, std::vector<double>());
    check_size(core_init_cov_ba.size(), 3);
    core_init_cov_ba_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_ba.data());

    // GPS Settings
    std::vector<double> gps1_pos_meas_noise;
    nh.param("gps1_pos_meas_noise", gps1_pos_meas_noise, std::vector<double>());
    check_size(gps1_pos_meas_noise.size(), 3);
    gps1_pos_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_pos_meas_noise.data());

    std::vector<double> gps1_vel_meas_noise;
    nh.param("gps1_vel_meas_noise", gps1_vel_meas_noise, std::vector<double>());
    check_size(gps1_vel_meas_noise.size(), 3);
    gps1_vel_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_vel_meas_noise.data());

    gps1_use_dyn_meas_noise_ = nh.param<bool>("gps1_use_dyn_meas_noise", gps1_use_dyn_meas_noise_);

    std::vector<double> gps1_cal_ig;
    nh.param("gps1_cal_ig", gps1_cal_ig, std::vector<double>());
    check_size(gps1_cal_ig.size(), 3);
    gps1_cal_ig_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_cal_ig.data());

    std::vector<double> gps1_state_init_cov;
    nh.param("gps1_state_init_cov", gps1_state_init_cov, std::vector<double>());
    check_size(gps1_state_init_cov.size(), 3);
    gps1_state_init_cov_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_state_init_cov.data());

    // Magnetometer Settings
    mag1_normalize_ = nh.param<bool>("mag1_normalize", mag1_normalize_);
    nh.param("mag1_declination", mag1_declination_, double());

    std::vector<double> mag1_meas_noise;
    nh.param("mag1_meas_noise", mag1_meas_noise, std::vector<double>());
    check_size(mag1_meas_noise.size(), 3);
    mag1_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(mag1_meas_noise.data());

    mag1_use_dyn_meas_noise_ = nh.param<bool>("mag1_use_dyn_meas_noise", mag1_use_dyn_meas_noise_);

    std::vector<double> mag1_cal_q_im;
    nh.param("mag1_cal_q_im", mag1_cal_q_im, std::vector<double>());
    check_size(mag1_cal_q_im.size(), 4);
    mag1_cal_q_im_ = Eigen::Quaterniond(mag1_cal_q_im[0], mag1_cal_q_im[1], mag1_cal_q_im[2], mag1_cal_q_im[3]);

    std::vector<double> mag1_state_init_cov;
    nh.param("mag1_state_init_cov", mag1_state_init_cov, std::vector<double>());
    check_size(mag1_state_init_cov.size(), 6);
    mag1_state_init_cov_ = Eigen::Map<Eigen::Matrix<double, 6, 1> >(mag1_state_init_cov.data());
  }
};

///
/// \brief The MarsWrapperGpsMag class MaRS GPS and magnetometer node
///
class MarsWrapperGpsMag
{
public:
  MarsWrapperGpsMag(ros::NodeHandle nh);

  using GpsCoordMsgFilter = message_filters::Subscriber<sensor_msgs::NavSatFix>;
  using GpsVelMsgFilter = message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>;
  using GpsMeasSyncFilter =
      message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped>;
  using ApproxTimePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
                                                                           geometry_msgs::TwistWithCovarianceStamped>;
  using GpsMeasApproxSyncFilter = message_filters::Synchronizer<ApproxTimePolicy>;

  // Node services
  ros::ServiceServer initialization_service_;  ///< Service handle for filter initialization

  ParamLoad m_sett_;

  ///
  /// \brief initServiceCallback Service to initialize / re-initialize the filter
  /// \return True if service call was successful
  ///
  bool initServiceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  // Dynamic reconfigure components
  dynamic_reconfigure::Server<mars_ros::marsConfig> reconfigure_srv_;
  dynamic_reconfigure::Server<mars_ros::marsConfig>::CallbackType reconfigure_cb_;

  ///
  /// \brief configCallback Callback to set all dynamic configurations
  /// \param config
  /// \param level
  ///
  void configCallback(mars_ros::marsConfig& config, uint32_t level);

  bool common_gps_ref_is_set_{ false };  ///< Indicator that the common reference was set
  Eigen::Vector3d p_wi_init_;            ///< Latest position which will be used to initialize the filter
  Eigen::Quaterniond q_wi_init_;         ///< Latest orientation to initialize the filter
  bool do_state_init_{ false };          ///< Trigger if the filter should be initialized

  ///
  /// \brief init used by the reconfigure GUI to reset the the buffer and sensors
  /// \return
  ///
  bool init();

  ///
  /// \brief set_common_gps_reference called on each GPS measurement update
  /// \param reference
  ///
  /// This function sets common GPS reference coordinates for all GPS
  ///
  void set_common_gps_reference(const mars::GpsCoordinates& reference);

  // Initialize framework components
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr_;  ///< Propagation sensor instance
  std::shared_ptr<mars::CoreState> core_states_sptr_;      ///< Core State instance
  mars::CoreLogic core_logic_;                             ///< Core Logic instance

  // Sensor instances
  std::shared_ptr<mars::GpsVelSensorClass> gps1_sensor_sptr_;  ///< GPS 1 sensor instance
  std::shared_ptr<mars::MagSensorClass> mag1_sensor_sptr_;     ///< MAG 1 sensor instance

  // Subscriber
  ros::Subscriber sub_imu_measurement_;     ///< IMU measurement subscriber
  ros::Subscriber sub_mag1_measurement_;    ///< MAG 1 MagneticField measurement subscriber
  GpsCoordMsgFilter sub_gps1_coord_meas_;   ///< GPS 1 NavSatFix measurement subscriber
  GpsVelMsgFilter sub_gps1_vel_meas_;       ///< GPS 1 TwistWithCovarianceStamped measurement subscriber
  GpsMeasApproxSyncFilter sync_gps1_meas_;  ///< GPS 1 Measurement Synchronizer Approximate time

  mars::MagnetometerInit mag_init_;

  // Sensor Callbacks
  ///
  /// \brief ImuMeasurementCallback IMU measurment callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the propagation sensor routine
  ///
  void ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas);

  ///
  /// \brief Gps1MeasurementCallback GPS 1 measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the GpsMeasurementUpdate routine. Since this update
  /// involves GPS coordinates and Velocity as two separate but synchronized messages, the design choice was made to use
  /// the timestamp of the GPS coordinate message.
  ///
  void Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& coord_meas,
                               const geometry_msgs::TwistWithCovarianceStampedConstPtr& vel_meas);

  ///
  /// \brief Mag1MeasurementCallback MAG 1 measurement callback
  /// \param meas
  ///
  void Mag1MeasurementCallback(const sensor_msgs::MagneticFieldConstPtr& meas);

  // Publisher
  ros::Publisher pub_ext_core_state_;       ///< Publisher for the Core-State mars_ros::ExtCoreState message
  ros::Publisher pub_core_pose_state_;      ///< Publisher for the Core-State pose stamped message
  ros::Publisher pub_ext_core_state_lite_;  ///< Publisher for the Core-State mars_ros::ExtCoreStateLite message
  ros::Publisher pub_core_odom_state_;      ///< Publisher for the Core-State as Odometry message
  ros::Publisher pub_core_path_;            ///< Publisher for all Core-States in buffer as path message
  ros::Publisher pub_gps1_state_;           ///< Publisher for the GPS 1 sensor calibration state
  ros::Publisher pub_gps1_enu_odom_;        ///< Publisher for the GPS1 ENU position Odometry message
  MarsPathGen path_generator_;              ///< Generator and storage for nav_msgs::Path

  ros::Publisher pub_mag1_state_;  ///< Publisher for the MAG 1 sensor calibration state

  // Publish groups
  ///
  /// \brief RunCoreStatePublisher Runs on each update sensor routine to publish the core-state
  ///
  /// This publishes the ExtCoreState and the Pose core state
  ///
  void RunCoreStatePublisher();

  // Sensor Updates
  ///
  /// \brief GpsVelMeasurementUpdate Generic GPS sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param gps_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  bool GpsVelMeasurementUpdate(std::shared_ptr<mars::GpsVelSensorClass> sensor_sptr,
                               const mars::GpsVelMeasurementType& gps_meas, const mars::Time& timestamp);
  ///
  /// \brief MagMeasurementUpdate
  /// \param sensor_sptr
  /// \param gps_meas
  /// \param timestamp
  ///
  bool MagMeasurementUpdate(std::shared_ptr<mars::MagSensorClass> sensor_sptr, const mars::MagMeasurementType& gps_meas,
                            const mars::Time& timestamp);
};

#endif  // MARS_WRAPPER_GPS_MAG_H
