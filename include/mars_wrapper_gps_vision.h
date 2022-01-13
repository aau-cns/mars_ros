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

#ifndef MARSWRAPPERDUALPOSE_H
#define MARSWRAPPERDUALPOSE_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mars/buffer.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#ifndef GPS_W_VEL
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_class.h>
#else
#include <geometry_msgs/TwistStamped.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_measurement_type.h>
#include <mars/sensors/gps_w_vel/gps_w_vel_sensor_class.h>
#endif
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/sensors/mag/mag_measurement_type.h>
#include <mars/sensors/mag/mag_sensor_class.h>
#include <mars/sensors/mag/mag_utils.h>
#include <mars/sensors/pressure/pressure_measurement_type.h>
#include <mars/sensors/pressure/pressure_sensor_class.h>
#include <mars/sensors/pressure/pressure_utils.h>
#include <mars/sensors/vision/vision_measurement_type.h>
#include <mars/sensors/vision/vision_sensor_class.h>
#include <mars_ros/marsConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/SetBool.h>

#include <boost/bind/bind.hpp>

#include "params/mars_dualpose_paramloader.h"

#define APPROX_TIME_SYNC

///
/// \brief The MarsWrapperGpsVision class for MaRS fusing GPS (pos or pos+vel), vision sensor, pressure, and
/// magnetometer.
///
/// The GPS frame is used as navigation frame, upon receiving the first vision measurement, the calibration between the
/// navigation frame and vision frame is calculated and the corresponding state of the vision sensor (T_vw) initialized.
///
class MarsWrapperGpsVision
{
public:
  MarsWrapperGpsVision(ros::NodeHandle nh);

#ifdef GPS_W_VEL
  using GpsCoordMsgFilter = message_filters::Subscriber<sensor_msgs::NavSatFix>;
  using GpsVelMsgFilter = message_filters::Subscriber<geometry_msgs::TwistStamped>;
  using GpsMeasSyncFilter = message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, geometry_msgs::TwistStamped>;
  using ApproxTimePolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistStamped>;
  using GpsMeasApproxSyncFilter = message_filters::Synchronizer<ApproxTimePolicy>;
#endif

  // Node services
  ros::ServiceServer initialization_service_;  ///< Service handle for filter initialization

  ParamLoad m_sett;

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

  // variables indicating initialization
  bool common_gps_ref_is_set_{ false };       ///< Indicator that the common reference was set
  bool common_pressure_ref_is_set_{ false };  ///< Indicator that the common reference was set
  bool have_pose1_{ false };                  //!< Indicator that a pose from sensor one (initial state) was received
  bool have_pose2_{ false };                  //!< Indicator that a pose from sensor one (initial state) was received
  Eigen::Vector3d p_wi_init_;                 ///< Latest position which will be used to initialize the filter
  Eigen::Quaterniond q_wi_init_;              ///< Latest orientation to initialize the filter
  mars::Buffer gps_meas_buffer_{ 100 };       ///!< Bufffer for GPS measurements to average reference frame
  mars::PoseSensorData pose12_calibration_;

  mars::PressureInit press_init_;
  mars::MagnetometerInit mag_init_;

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
  void set_common_gps_reference(const mars::GpsCoordinates& reference, const mars::Time& timestamp);

  void set_common_pressure_reference(const mars::Pressure& reference, const mars::Time& timestamp);

  // Initialize framework components
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr_;  ///< Propagation sensor instance
  std::shared_ptr<mars::CoreState> core_states_sptr_;      ///< Core State instance
  mars::CoreLogic core_logic_;                             ///< Core Logic instance

  // Sensor instances
  std::shared_ptr<mars::VisionSensorClass> vision1_sensor_sptr_;     /// Pose1 update sensor instance
  std::shared_ptr<mars::PressureSensorClass> pressure_sensor_sptr_;  /// Pressure update sensor instance
  std::shared_ptr<mars::MagSensorClass> mag1_sensor_sptr_;           ///< MAG 1 sensor instance

#ifndef GPS_W_VEL
  std::shared_ptr<mars::GpsSensorClass> gps1_sensor_sptr_;  /// GPS 1 sensor instance
#else
  std::shared_ptr<mars::GpsVelSensorClass> gps1_sensor_sptr_;  ///< GPS 1 sensor instance
#endif

  // Subscriber
  ros::Subscriber sub_imu_measurement_;       ///< IMU measurement subscriber
  ros::Subscriber sub_pose1_measurement_;     ///< Pose stamped measurement subscriber
  ros::Subscriber sub_pressure_measurement_;  ///< Pressure stamped measurement subscriber
  ros::Subscriber sub_mag1_measurement_;      ///< MAG 1 MagneticField measurement subscriber

#ifndef GPS_W_VEL
  ros::Subscriber sub_gps1_measurement_;  ///< GPS 1 NavSatFixConstPtr measurement subscriber
#else
  GpsCoordMsgFilter sub_gps1_coord_meas_;                      ///< GPS 1 NavSatFix measurement subscriber
  GpsVelMsgFilter sub_gps1_vel_meas_;  ///< GPS 1 TwistWithCovarianceStamped measurement subscriber

#ifndef APPROX_TIME_SYNC
  GpsMeasSyncFilter sync_gps1_meas_;  ///< GPS 1 Measurement Synchronizer exact time
#else
  GpsMeasApproxSyncFilter sync_gps1_meas_;  ///< GPS 1 Measurement Synchronizer Approximate time
#endif  // APPROX_TIME_SYNC
#endif  // GPS_W_VEL

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
  void Pose1MeasurementCallback(const geometry_msgs::PoseStampedConstPtr& meas);

#ifndef GPS_W_VEL
  ///
  /// \brief Gps1MeasurementCallback GPS 1 measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the GpsMeasurementUpdate routine
  ///
  void Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas);
#else
  ///
  /// \brief Gps1MeasurementCallback GPS 1 measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the GpsMeasurementUpdate routine. Since this update
  /// involves GPS coordinates and Velocity as two separate but synchronized messages, the design choice was made to use
  /// the timestamp of the GPS coordinate message.
  ///
  void Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& coord_meas,
                               const geometry_msgs::TwistStampedConstPtr& vel_meas);
#endif

  ///
  /// \brief PressureMeasurementCallback Pressure measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running a PressureMeasurementUpdate routine
  ///
  void PressureMeasurementCallback(const sensor_msgs::FluidPressureConstPtr& meas);

  ///
  /// \brief Mag1MeasurementCallback MAG 1 measurement callback
  /// \param meas
  ///
  void Mag1MeasurementCallback(const sensor_msgs::MagneticFieldConstPtr& meas);

  // Publisher
  ros::Publisher pub_ext_core_state_;       ///< Publisher for the Core-State mars_ros::ExtCoreState message
  ros::Publisher pub_ext_core_state_lite_;  ///< Publisher for the Core-State mars_ros::ExtCoreStateLite message
  ros::Publisher pub_core_pose_state_;      ///< Publisher for the Core-State pose stamped message
  ros::Publisher pub_core_odom_state_;      ///< Publisher for the Core-State odom stamped message
  ros::Publisher pub_pose1_state_;          ///< Publisher for the pose sensor calibration state
  ros::Publisher pub_pose2_state_;          ///< Publisher for the pose sensor calibration state
  ros::Publisher pub_gps1_state_;           ///< Publisher for the GPS 1 sensor calibration state
  ros::Publisher pub_press_state_;          ///< Publisher for the pressure sensor calibration state

  ros::Publisher pub_gps1_enu_odom_;      ///< Publisher for the GPS1 ENU position Odometry message
  ros::Publisher pub_baro1_height_vec3_;  ///< Publisher for the GPS1 ENU position Odometry message

  uint32_t pub_prob_cnt{ 0 };

  // Publish groups
  ///
  /// \brief RunCoreStatePublisher Runs on each update sensor routine to publish the core-state
  ///
  /// This publishes the ExtCoreState and the Pose core state
  ///
  void RunCoreStatePublisher();

#ifndef GPS_W_VEL
  ///
  /// \brief GpsMeasurementUpdate Generic GPS sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param gps_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  bool GpsMeasurementUpdate(std::shared_ptr<mars::GpsSensorClass> sensor_sptr, const mars::GpsMeasurementType& gps_meas,
                            const mars::Time& timestamp);
#else
  ///
  /// \brief GpsVelMeasurementUpdate Generic GPS sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param gps_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  bool GpsVelMeasurementUpdate(std::shared_ptr<mars::GpsVelSensorClass> sensor_sptr,
                               const mars::GpsVelMeasurementType& gps_meas, const mars::Time& timestamp);
#endif

  ///
  /// \brief PressureMeasurementUpdate Generic pressure sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param press_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  bool PressureMeasurementUpdate(std::shared_ptr<mars::PressureSensorClass> sensor_sptr,
                                 const mars::PressureMeasurementType& press_meas, const mars::Time& timestamp);

  ///
  /// \brief MagMeasurementUpdate
  /// \param sensor_sptr
  /// \param gps_meas
  /// \param timestamp
  ///
  bool MagMeasurementUpdate(std::shared_ptr<mars::MagSensorClass> sensor_sptr, const mars::MagMeasurementType& gps_meas,
                            const mars::Time& timestamp);

  ///
  /// \brief PressureMeasurementUpdate Generic pressure sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param press_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  bool VisionMeasurementUpdate(std::shared_ptr<mars::VisionSensorClass> sensor_sptr,
                               const mars::VisionMeasurementType& vision_meas, const mars::Time& timestamp);
};

#endif  // MARSWRAPPERDUALPOSE_H
