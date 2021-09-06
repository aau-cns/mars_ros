// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARSWRAPPERPOSE_H
#define MARSWRAPPERPOSE_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/pose/pose_measurement_type.h>
#include <mars/sensors/pose/pose_sensor_class.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>

#include <dynamic_reconfigure/server.h>
#include <mars_ros/marsConfig.h>
#include <boost/bind/bind.hpp>

///
/// \brief The MarsWrapperPose class MaRS single pose node
///
class MarsWrapperPose
{
public:
  MarsWrapperPose(ros::NodeHandle nh);

  // Settings
  bool publish_on_propagation_{ false };  ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };        ///< Set to true to use rostime now for all sensor updates
  bool verbose_output_{ false };          ///< If true, all verbose infos are printed
  bool verbose_ooo_{ false };             ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };   ///< If true, all out of order propagation sensor meas are discarded

  uint32_t pub_cb_buffer_size_{ 1 };         ///< Callback buffersize for all outgoing topics
  uint32_t sub_imu_cb_buffer_size_{ 200 };   ///< Callback buffersize for propagation sensor measurements
  uint32_t sub_sensor_cb_buffer_size_{ 1 };  ///< Callback buffersize for all non-propagation sensor measurements

  // Node services
  ros::ServiceServer initialization_service_;  ///< Service handle for filter initialization

  ///
  /// \brief initServiceCallback Service to initialize / re-initialize the filter
  /// \return True if service call was successful
  ///
  bool initServiceCallback(std_srvs::SetBool::Request& /*request*/, std_srvs::SetBool::Response& /*response*/);

  // Dynamic reconfigure components
  dynamic_reconfigure::Server<mars_ros::marsConfig> reconfigure_srv_;
  dynamic_reconfigure::Server<mars_ros::marsConfig>::CallbackType reconfigure_cb_;

  ///
  /// \brief configCallback Callback to set all dynamic configurations
  /// \param config
  /// \param level
  ///
  void configCallback(mars_ros::marsConfig& config, uint32_t level);

  Eigen::Vector3d p_wi_init_;     ///< Latest position which will be used to initialize the filter
  Eigen::Quaterniond q_wi_init_;  ///< Latest orientation to initialize the filter

  ///
  /// \brief init used by the reconfigure GUI to reset the the buffer and sensors
  /// \return
  ///
  bool init();

  // Initialize framework components
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr_;  ///< Propagation sensor instance
  std::shared_ptr<mars::CoreState> core_states_sptr_;      ///< Core State instance
  mars::CoreLogic core_logic_;                             ///< Core Logic instance

  // Sensor instances
  std::shared_ptr<mars::PoseSensorClass> pose1_sensor_sptr_;  /// Pose update sensor instance

  // Subscriber
  ros::Subscriber sub_imu_measurement_;            ///< IMU measurement subscriber
  ros::Subscriber sub_pose_measurement_;           ///< Pose stamped measurement subscriber
  ros::Subscriber sub_pose_with_cov_measurement_;  ///< Pose with covariance stamped subscriber
  ros::Subscriber sub_odom_measurement_;           ///< Odometry measurement subscriber
  ros::Subscriber sub_transform_measurement_;      ///< Transform stamped measurement subscriber

  // Sensor Callbacks
  ///
  /// \brief ImuMeasurementCallback IMU measurment callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the propagation sensor routine
  ///
  void ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas);

  ///
  /// \brief PoseMeasurementCallback Pose measurement callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the PoseMeasurementUpdate routine
  ///
  void PoseMeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas);

  ///
  /// \brief PoseWithCovMeasurementCallback Pose with cov measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the PoseMeasurementUpdate routine
  ///
  void PoseWithCovMeasurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& meas);

  ///
  /// \brief TransformMeasurementCallback Transform measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the PoseMeasurementUpdate routine
  ///
  void TransformMeasurementCallback(const geometry_msgs::TransformStampedConstPtr& meas);

  ///
  /// \brief OdomMeasurementCallback Odometry measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the PoseMeasurementUpdate routine
  ///
  void OdomMeasurementCallback(const nav_msgs::OdometryConstPtr& meas);

  // Publisher
  ros::Publisher pub_ext_core_state_;       ///< Publisher for the Core-State mars_ros::ExtCoreState message
  ros::Publisher pub_ext_core_state_lite_;  ///< Publisher for the Core-State mars_ros::ExtCoreStateLite message
  ros::Publisher pub_core_pose_state_;      ///< Publisher for the Core-State pose stamped message
  ros::Publisher pub_pose1_state_;          ///< Publisher for the pose sensor calibration state

  // Publish groups
  ///
  /// \brief RunCoreStatePublisher Runs on each update sensor routine to publish the core-state
  ///
  /// This publishes the ExtCoreState and the Pose core state
  ///
  void RunCoreStatePublisher();

  // Sensor Updates
  ///
  /// \brief PoseMeasurementUpdate Pose sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param pose_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  void PoseMeasurementUpdate(std::shared_ptr<mars::PoseSensorClass> sensor_sptr,
                             const mars::PoseMeasurementType& pose_meas, const mars::Time& timestamp);
};

#endif  // MARSWRAPPERPOSE_H
