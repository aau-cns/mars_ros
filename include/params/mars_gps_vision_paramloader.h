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

#ifndef MARSGPSVISION_LOADER_H
#define MARSGPSVISION_LOADER_H

#include <ros/node_handle.h>

#include <Eigen/Dense>

#define PARAM_PRINTER(args) std::cout << "[ParamLoader] " << args;

class ParamLoad
{
public:
  //  bool publish_on_propagation_{ false };   ///< Set true to publish the core state on propagation
  //  bool use_ros_time_now_{ false };         ///< Set to true to use rostime now for all sensor updates
  //  bool gps_ros_time_now_{ false };         ///< Set to true to use rostime now for all sensor updates
  //  bool verbose_output_{ false };           ///< If true, all verbose infos are printed
  //  bool verbose_ooo_{ false };              ///< If true, only out of order verbose msgs are printed
  //  bool discard_ooo_prop_meas_{ false };    ///< If true, all out of order propagation sensor meas are discarded
  //  bool use_common_gps_reference_{ true };  ///< Use a common GPS reference for all sensors
  //  bool pub_cov_{ true };                   ///< Publish covariances in the ext core state message if true
  //  uint32_t buffer_size_{ 2000 };           ///< Set mars buffersize
  bool publish_on_propagation_{ true };  ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };       ///< Set to true to use rostime now for all sensor updates
  bool gps_ros_time_now_{ false };       ///< Set to true to use rostime now for GPS sensor updates
  bool verbose_output_{ false };         ///< If true, all verbose infos are printed
  bool verbose_ooo_{ true };             ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };  ///< If true, all out of order propagation sensor meas are discarded
  bool pub_cov_{ true };                 ///< Publish covariances in the ext core state message if true
  uint32_t buffer_size_{ 2000 };         ///< Set mars buffersize

  bool use_tcpnodelay_{ true };  ///< Use tcp no delay for the ROS msg. system
  bool bypass_init_service_{ false };

  // enablers
  bool use_pressure_{ true };       ///< Determines if the pressure sensor is used
  bool use_magnetometer_{ false };  ///< Determines if the magnetometer is used

  // initializers
  bool use_common_gps_reference_{ true };  ///< Use a common GPS reference for all sensors
  bool enable_manual_yaw_init_{ false };   ///< Initialize the yaw based on 'yaw_init_deg_'
  double yaw_init_deg_{ 0 };               ///< Yaw for core state init if 'enable_manual_yaw_init_' is true
  int auto_mag_init_samples_{ 30 };        ///< Number if meas. sample if auto init is used
                                           ///(enable_manual_yaw_init_=false)

  // ros interface
  uint32_t pub_cb_buffer_size_{ 1 };         ///< Callback buffersize for all outgoing topics
  uint32_t sub_imu_cb_buffer_size_{ 200 };   ///< Callback buffersize for propagation sensor measurements
  uint32_t sub_sensor_cb_buffer_size_{ 1 };  ///< Callback buffersize for all non-propagation sensor measurements
  uint32_t pub_prop_divider_{ 4 };           ///< Divider of propagation rate for publishing

  // publishers
  bool publish_gps_enu_{ false };     ///< Publish GPS as ENU in the ref. frame used by the filter
  bool publish_baro_height_{ true };  ///< Publish pressure-based height as vector in the ref. frame used by the filter

  // IMU noise
  double g_rate_noise_;
  double g_bias_noise_;
  double a_noise_;
  double a_bias_noise_;

  // core init covariances
  Eigen::Vector3d core_init_cov_p_;
  Eigen::Vector3d core_init_cov_v_;
  Eigen::Vector3d core_init_cov_q_;
  Eigen::Vector3d core_init_cov_bw_;
  Eigen::Vector3d core_init_cov_ba_;

  void printCore()
  {
    PARAM_PRINTER("core_init_cov_p_:          " << core_init_cov_p_.transpose() << "\n");
    PARAM_PRINTER("core_init_cov_v_:          " << core_init_cov_v_.transpose() << "\n");
    PARAM_PRINTER("core_init_cov_q_:          " << core_init_cov_q_.transpose() << "\n");
    PARAM_PRINTER("core_init_cov_bw_:         " << core_init_cov_bw_.transpose() << "\n");
    PARAM_PRINTER("core_init_cov_ba_:         " << core_init_cov_ba_.transpose() << "\n");
  }

  // Pose1 calibration
  bool vision1_fixed_scale_;
  Eigen::Vector3d vision1_pos_meas_noise_;
  Eigen::Vector3d vision1_att_meas_noise_;
  Eigen::Vector3d vision1_cal_p_ip_;
  Eigen::Quaterniond vision1_cal_q_ip_;
  Eigen::Matrix<double, 6, 1> vision1_cal_ip_init_cov_;
  Eigen::Matrix<double, 1, 1> vision1_scale_init_cov_;

  void printPose1()
  {
    PARAM_PRINTER("pose1_pos_meas_noise_:     " << vision1_pos_meas_noise_.transpose() << "\n");
    PARAM_PRINTER("pose1_att_meas_noise_:     " << vision1_att_meas_noise_.transpose() << "\n");
    PARAM_PRINTER("pose1_cal_p_ip_:           " << vision1_cal_p_ip_.transpose() << "\n");
    PARAM_PRINTER("pose1_cal_q_ip_:           " << vision1_cal_q_ip_.w() << " " << vision1_cal_q_ip_.vec().transpose()
                                                << "\n");
    PARAM_PRINTER("pose1_state_init_cov_:     " << vision1_cal_ip_init_cov_.transpose() << "\n");

    if (vision1_fixed_scale_)
    {
      PARAM_PRINTER("vision1_fixed_scale_:      True\n");
    }
    else
    {
      PARAM_PRINTER("vision1_fixed_scale_:      False\n");
      PARAM_PRINTER("vision1_scale_init_cov_:   " << vision1_scale_init_cov_.transpose() << "\n");
    }
  }

  // GPS1 calibration
  Eigen::Vector3d gps1_pos_meas_noise_;
  Eigen::Vector3d gps1_vel_meas_noise_;
  Eigen::Vector3d gps1_cal_ig_;
  Eigen::Vector3d gps1_state_init_cov_;

  void printGps1()
  {
    PARAM_PRINTER("gps1_pos_meas_noise_:      " << gps1_pos_meas_noise_.transpose() << "\n");
    PARAM_PRINTER("gps1_vel_meas_noise_:      " << gps1_vel_meas_noise_.transpose() << "\n");
    PARAM_PRINTER("gps1_cal_ig_:              " << gps1_cal_ig_.transpose() << "\n");
    PARAM_PRINTER("gps1_state_init_cov_:      " << gps1_state_init_cov_.transpose() << "\n");
  }

  // Pressure calibration
  double pressure1_meas_noise_;
  Eigen::Vector3d pressure1_cal_ip_;
  Eigen::Vector3d pressure1_state_init_cov_;

  bool pressure1_const_temp_{ true };
  double pressure1_temp_K_{ 293.15 };
  double pressure1_init_duration_{ 1.0 };

  void printPressure1()
  {
    if (use_pressure_)
    {
      PARAM_PRINTER("pressure1_meas_noise_:     " << pressure1_meas_noise_ << "\n");
      PARAM_PRINTER("pressure1_cal_ip_:         " << pressure1_cal_ip_.transpose() << "\n");
      PARAM_PRINTER("pressure1_state_init_cov_: " << pressure1_state_init_cov_.transpose() << "\n");
      PARAM_PRINTER("pressure1_const_temp_:     " << pressure1_const_temp_ << "\n");
      PARAM_PRINTER("pressure1_temp_K_:         " << pressure1_temp_K_ << "\n");
      PARAM_PRINTER("pressure1_init_duration_:  " << pressure1_init_duration_ << "\n");
    }
    else
    {
      PARAM_PRINTER("use_pressure_:             false");
    }
  }

  // Magnetometer calibration
  bool mag1_normalize_{ true };
  double mag1_declination_{ 0.0 };
  Eigen::Vector3d mag1_meas_noise_;
  Eigen::Quaterniond mag1_cal_q_im_;
  Eigen::Matrix<double, 6, 1> mag1_state_init_cov_;

  void printMag()
  {
    if (use_magnetometer_)
    {
      PARAM_PRINTER("mag1_normalize_:           " << mag1_normalize_ << "\n");
      PARAM_PRINTER("mag1_declination_:         " << mag1_declination_ << "\n");
      PARAM_PRINTER("mag1_meas_noise_:          " << mag1_meas_noise_.transpose() << "\n");
      PARAM_PRINTER("mag1_cal_q_im_:            " << mag1_cal_q_im_.coeffs() << "\n");
      PARAM_PRINTER("mag1_state_init_cov_:      " << mag1_state_init_cov_.transpose() << "\n");
    }
    else
    {
      PARAM_PRINTER("use_magnetometer_:         false");
    }
  }

  void printAll()
  {
    std::cout << "[ParamLoader] printing full parameter list:" << std::endl;
    printCore();
    printPose1();
    printGps1();
    printPressure1();
    printMag();
    std::cout << std::endl;
  }

  void check_size(const int& size_in, const int& size_comp)
  {
    if (size_comp != size_in)
    {
      std::cerr << "YAML array with wrong size" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  template <int _Rows>
  void check_and_load(Eigen::Matrix<double, _Rows, 1>& vec, const ros::NodeHandle nh, const std::string& name)
  {
    std::vector<double> tmp_vec;
    nh.param(name, tmp_vec, std::vector<double>());
    if (tmp_vec.size() != _Rows)
    {
      std::cerr << "[ParamLoader] " << name << ": YAML array with wrong size (which is " << tmp_vec.size() << ")"
                << std::endl;
    }
    vec = Eigen::Map<Eigen::Matrix<double, _Rows, 1> >(tmp_vec.data());
  }

  void check_size_4(const int& size)
  {
    if (size != 4)
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
    gps_ros_time_now_ = nh.param<bool>("gps_ros_time_now", gps_ros_time_now_);
    verbose_output_ = nh.param<bool>("verbose", verbose_output_);
    verbose_ooo_ = nh.param<bool>("verbose_out_of_order", verbose_ooo_);
    discard_ooo_prop_meas_ = nh.param<bool>("discard_ooo_prop_meas", discard_ooo_prop_meas_);
    pub_cov_ = nh.param<bool>("pub_cov", pub_cov_);
    buffer_size_ = nh.param<int>("buffer_size", buffer_size_);

    // Yaw initialization
    use_pressure_ = nh.param<bool>("use_pressure", use_pressure_);
    use_magnetometer_ = nh.param<bool>("use_magnetometer", use_magnetometer_);
    enable_manual_yaw_init_ = nh.param<bool>("enable_manual_yaw_init", enable_manual_yaw_init_);
    nh.param("yaw_init_deg", yaw_init_deg_, double());
    auto_mag_init_samples_ = uint32_t(nh.param<int>("auto_mag_init_samples", int(auto_mag_init_samples_)));

    // ROS Settings
    use_tcpnodelay_ = nh.param<bool>("use_tcpnodelay", use_tcpnodelay_);
    bypass_init_service_ = nh.param<bool>("bypass_init_service", bypass_init_service_);

    use_common_gps_reference_ = nh.param<bool>("use_common_gps_reference", use_common_gps_reference_);

    pub_cb_buffer_size_ = uint32_t(nh.param<int>("pub_cb_buffer_size", int(pub_cb_buffer_size_)));
    sub_imu_cb_buffer_size_ = uint32_t(nh.param<int>("sub_imu_cb_buffer_size", int(sub_imu_cb_buffer_size_)));
    sub_sensor_cb_buffer_size_ = uint32_t(nh.param<int>("sub_sensor_cb_buffer_size", int(sub_sensor_cb_buffer_size_)));
    pub_prop_divider_ = uint32_t(nh.param<int>("pub_prop_divider", int(pub_prop_divider_)));

    // Publisher
    publish_gps_enu_ = nh.param<bool>("publish_gps_enu", publish_gps_enu_);
    publish_baro_height_ = nh.param<bool>("publish_baro_height", publish_baro_height_);

    // IMU parameter
    nh.param("gyro_rate_noise", g_rate_noise_, double());
    nh.param("gyro_bias_noise", g_bias_noise_, double());
    nh.param("acc_noise", a_noise_, double());
    nh.param("acc_bias_noise", a_bias_noise_, double());

    // Core state covariance
    check_and_load<3>(core_init_cov_p_, nh, "core_init_cov_p");
    check_and_load<3>(core_init_cov_v_, nh, "core_init_cov_v");
    check_and_load<3>(core_init_cov_q_, nh, "core_init_cov_q");
    check_and_load<3>(core_init_cov_bw_, nh, "core_init_cov_bw");
    check_and_load<3>(core_init_cov_ba_, nh, "core_init_cov_ba");

    // Vision1 Settings
    nh.param("vision1_fixed_scale", vision1_fixed_scale_, true);
    check_and_load<3>(vision1_pos_meas_noise_, nh, "vision1_pos_meas_noise");
    check_and_load<3>(vision1_att_meas_noise_, nh, "vision1_att_meas_noise");
    check_and_load<3>(vision1_cal_p_ip_, nh, "vision1_cal_p_ip");

    std::vector<double> vision1_cal_q_ip;
    nh.param("vision1_cal_q_ip", vision1_cal_q_ip, std::vector<double>());
    check_size_4(vision1_cal_q_ip.size());
    vision1_cal_q_ip_.w() = vision1_cal_q_ip.at(0);
    vision1_cal_q_ip_.x() = vision1_cal_q_ip.at(1);
    vision1_cal_q_ip_.y() = vision1_cal_q_ip.at(2);
    vision1_cal_q_ip_.z() = vision1_cal_q_ip.at(3);
    check_and_load<6>(vision1_cal_ip_init_cov_, nh, "vision1_cal_ip_init_cov");
    if (!vision1_fixed_scale_)
      check_and_load<1>(vision1_scale_init_cov_, nh, "vision1_scale_init_cov");
    else
      vision1_scale_init_cov_ = Eigen::Matrix<double, 1, 1>(1e-15);

    // GPS Settings
    check_and_load<3>(gps1_pos_meas_noise_, nh, "gps1_pos_meas_noise");
#ifdef GPS_W_VEL
    check_and_load<3>(gps1_vel_meas_noise_, nh, "gps1_vel_meas_noise");
#endif
    check_and_load<3>(gps1_cal_ig_, nh, "gps1_cal_ig");
    check_and_load<3>(gps1_state_init_cov_, nh, "gps1_state_init_cov");

    // Pressure Settings
    pressure1_meas_noise_ = nh.param<double>("pressure1_meas_noise", 1.0);
    pressure1_const_temp_ = nh.param<bool>("pressure1_const_temp", pressure1_const_temp_);
    pressure1_temp_K_ = nh.param<double>("pressure1_temp_K", pressure1_temp_K_);
    pressure1_init_duration_ = nh.param<double>("pressure1_init_duration", pressure1_init_duration_);
    check_and_load<3>(pressure1_cal_ip_, nh, "pressure1_cal_ip");
    check_and_load<3>(pressure1_state_init_cov_, nh, "pressure1_state_init_cov");

    // Magnetometer Settings
    mag1_normalize_ = nh.param<bool>("mag1_normalize", mag1_normalize_);
    nh.param("mag1_declination", mag1_declination_, double());

    check_and_load<3>(mag1_meas_noise_, nh, "mag1_meas_noise");

    std::vector<double> mag1_cal_q_im;
    nh.param("mag1_cal_q_im", mag1_cal_q_im, std::vector<double>());
    check_size(mag1_cal_q_im.size(), 4);
    mag1_cal_q_im_ = Eigen::Quaterniond(mag1_cal_q_im[0], mag1_cal_q_im[1], mag1_cal_q_im[2], mag1_cal_q_im[3]);

    check_and_load<6>(mag1_state_init_cov_, nh, "mag1_state_init_cov");
    printAll();
  }

};  // class ParamLoad

#endif  // MARSGPSVISION_LOADER_H
