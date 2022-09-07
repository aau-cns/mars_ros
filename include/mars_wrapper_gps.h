// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARSWRAPPERGPS_H
#define MARSWRAPPERGPS_H

#include <dynamic_reconfigure/server.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_class.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars_msg_conv.h>
#include <mars_ros/marsConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
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

  bool publish_gps_enu_{ false };         ///< Publish GPS as ENU in the ref. frame used by the filter
  bool enable_manual_yaw_init_{ false };  ///< Initialize the yaw based on 'yaw_init_deg_'
  double yaw_init_deg_{ 0 };              ///< Yaw for core state init if 'enable_manual_yaw_init_' is true

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
  Eigen::Vector3d gps1_cal_ig_;
  Eigen::Vector3d gps1_state_init_cov_;

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
    publish_on_propagation_ = nh.param<bool>("pub_on_prop", publish_on_propagation_);
    use_ros_time_now_ = nh.param<bool>("use_ros_time_now", use_ros_time_now_);
    verbose_output_ = nh.param<bool>("verbose", verbose_output_);
    verbose_ooo_ = nh.param<bool>("verbose_out_of_order", verbose_ooo_);
    discard_ooo_prop_meas_ = nh.param<bool>("discard_ooo_prop_meas", discard_ooo_prop_meas_);
    pub_cov_ = nh.param<bool>("pub_cov", pub_cov_);
    pub_path_ = nh.param<bool>("pub_path", pub_path_);
    buffer_size_ = nh.param<int>("buffer_size", buffer_size_);

    use_tcpnodelay_ = nh.param<bool>("use_tcpnodelay", use_tcpnodelay_);
    bypass_init_service_ = nh.param<bool>("bypass_init_service", bypass_init_service_);

    pub_cb_buffer_size_ = uint32_t(nh.param<int>("pub_cb_buffer_size", int(pub_cb_buffer_size_)));
    sub_imu_cb_buffer_size_ = uint32_t(nh.param<int>("sub_imu_cb_buffer_size", int(sub_imu_cb_buffer_size_)));
    sub_sensor_cb_buffer_size_ = uint32_t(nh.param<int>("sub_sensor_cb_buffer_size", int(sub_sensor_cb_buffer_size_)));

    publish_gps_enu_ = nh.param<bool>("publish_gps_enu", publish_gps_enu_);
    enable_manual_yaw_init_ = nh.param<bool>("enable_manual_yaw_init", enable_manual_yaw_init_);
    nh.param("yaw_init_deg", yaw_init_deg_, double());

    nh.param("gyro_rate_noise", g_rate_noise_, double());
    nh.param("gyro_bias_noise", g_bias_noise_, double());
    nh.param("acc_noise", a_noise_, double());
    nh.param("acc_bias_noise", a_bias_noise_, double());

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

    // GPS Sensor
    std::vector<double> gps1_pos_meas_noise;
    nh.param("gps1_pos_meas_noise", gps1_pos_meas_noise, std::vector<double>());
    check_size(gps1_pos_meas_noise.size(), 3);
    gps1_pos_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_pos_meas_noise.data());

    std::vector<double> gps1_vel_meas_noise;
    nh.param("gps1_vel_meas_noise", gps1_vel_meas_noise, std::vector<double>());
    check_size(gps1_vel_meas_noise.size(), 3);
    gps1_vel_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_vel_meas_noise.data());

    std::vector<double> gps1_cal_ig;
    nh.param("gps1_cal_ig", gps1_cal_ig, std::vector<double>());
    check_size(gps1_cal_ig.size(), 3);
    gps1_cal_ig_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_cal_ig.data());

    std::vector<double> gps1_state_init_cov;
    nh.param("gps1_state_init_cov", gps1_state_init_cov, std::vector<double>());
    check_size(gps1_state_init_cov.size(), 3);
    gps1_state_init_cov_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_state_init_cov.data());
  }
};

///
/// \brief The MarsWrapperGps class MaRS single GPS node
///
class MarsWrapperGps
{
public:
  MarsWrapperGps(ros::NodeHandle nh);

  // Settings
  ParamLoad m_sett_;

  // Node services
  ros::ServiceServer initialization_service_;  ///< Service handle for filter initialization

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
  std::shared_ptr<mars::GpsSensorClass> gps1_sensor_sptr_;  ///< GPS 1 sensor instance

  // Subscriber
  ros::Subscriber sub_imu_measurement_;   ///< IMU measurement subscriber
  ros::Subscriber sub_gps1_measurement_;  ///< GPS 1 NavSatFixConstPtr measurement subscriber

  // Sensor Callbacks
  ///
  /// \brief ImuMeasurementCallback IMU measurment callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the propagation sensor routine
  ///
  void ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas);

  ///
  /// \brief GpsMeasurementCallback GPS 1 measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the GpsMeasurementUpdate routine
  ///
  void Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas);

  // Publisher
  ros::Publisher pub_ext_core_state_;       ///< Publisher for the Core-State mars_ros::ExtCoreState message
  ros::Publisher pub_core_pose_state_;      ///< Publisher for the Core-State pose stamped message
  ros::Publisher pub_core_odom_state_;      ///< Publisher for the Core-State as Odometry message
  ros::Publisher pub_ext_core_state_lite_;  ///< Publisher for the Core-State mars_ros::ExtCoreStateLite message
  ros::Publisher pub_core_path_;            ///< Publisher for all Core-States in buffer as path message
  MarsPathGen path_generator_;              ///< Generator and storage for nav_msgs::Path

  ros::Publisher pub_gps1_state_;     ///< Publisher for the GPS sensor calibration state
  ros::Publisher pub_gps1_enu_odom_;  ///< Publisher for the GPS ENU position Odometry message

  // Publish groups
  ///
  /// \brief RunCoreStatePublisher Runs on each update sensor routine to publish the core-state
  ///
  /// This publishes the ExtCoreState and the Pose core state
  ///
  void RunCoreStatePublisher();

  // Sensor Updates
  ///
  /// \brief GpsMeasurementUpdate Generic GPS sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param gps_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  void GpsMeasurementUpdate(std::shared_ptr<mars::GpsSensorClass> sensor_sptr, const mars::GpsMeasurementType& gps_meas,
                            const mars::Time& timestamp);
};

#endif  // MARSWRAPPERGPS_H
