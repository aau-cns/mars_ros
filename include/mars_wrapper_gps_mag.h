// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
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

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_sensor_class.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/mag/mag_measurement_type.h>
#include <mars/sensors/mag/mag_sensor_class.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>

#include <dynamic_reconfigure/server.h>
#include <mars_ros/marsConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind/bind.hpp>

///
/// \brief The MarsWrapperGpsMag class MaRS multi GPS node
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

  // Settings
  bool publish_on_propagation_{ false };   ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };         ///< Set to true to use rostime now for all sensor updates
  bool verbose_output_{ false };           ///< If true, all verbose infos are printed
  bool verbose_ooo_{ false };              ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };    ///< If true, all out of order propagation sensor meas are discarded
  bool use_common_gps_reference_{ true };  ///< Use a common GPS reference for all sensors

  uint32_t pub_cb_buffer_size_{ 1 };         ///< Callback buffersize for all outgoing topics
  uint32_t sub_imu_cb_buffer_size_{ 200 };   ///< Callback buffersize for propagation sensor measurements
  uint32_t sub_sensor_cb_buffer_size_{ 1 };  ///< Callback buffersize for all non-propagation sensor measurements

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
  ros::Publisher pub_gps1_state_;           ///< Publisher for the GPS 1 sensor calibration state
  ros::Publisher pub_mag1_state_;           ///< Publisher for the MAG 1 sensor calibration state

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
  void GpsVelMeasurementUpdate(std::shared_ptr<mars::GpsVelSensorClass> sensor_sptr,
                               const mars::GpsVelMeasurementType& gps_meas, const mars::Time& timestamp);
  ///
  /// \brief MagMeasurementUpdate
  /// \param sensor_sptr
  /// \param gps_meas
  /// \param timestamp
  ///
  void MagMeasurementUpdate(std::shared_ptr<mars::MagSensorClass> sensor_sptr, const mars::MagMeasurementType& gps_meas,
                            const mars::Time& timestamp);
};

#endif  // MARS_WRAPPER_GPS_MAG_H
