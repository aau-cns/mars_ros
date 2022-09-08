![MaRS ROS Logo](./resources/mars_ros_logo.png)

# Introduction

[![ROS-Noetic](https://github.com/aau-cns/mars_ros/actions/workflows/ros_noetic.yml/badge.svg)](https://github.com/aau-cns/mars_ros/actions/workflows/ros_noetic.yml) [![ROS-Melodic](https://github.com/aau-cns/mars_ros/actions/workflows/ros_melodic.yml/badge.svg)](https://github.com/aau-cns/mars_ros/actions/workflows/ros_melodic.yml) [![ROS-Kinetic](https://github.com/aau-cns/mars_ros/actions/workflows/ros_kinetic.yml/badge.svg)](https://github.com/aau-cns/mars_ros/actions/workflows/ros_kinetic.yml)<br/>
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5185909.svg)](https://doi.org/10.5281/zenodo.5185909) [![IEEE](https://img.shields.io/badge/IEEE-10.1109/LRA.2020.3043195-00498d.svg)](https://ieeexplore.ieee.org/document/9286578) [![License](https://img.shields.io/badge/License-AAUCNS-green.svg)](./LICENSE)

This package is a ROS Wrapper for the Modular and Robust State-Estimation (MaRS) Library, which can be found [here](https://github.com/aau-cns/mars_lib), with the technology described by this [publication](https://ieeexplore.ieee.org/document/9286578). The wrapper defines simple nodes for direct use. Additional sensors can be added to pre-existing nodes, or a dedicated ROS node can be designed for a specific application.

Please note that the general definition of a sensor module is defined within the MaRS library. The sensor modules defined by the MaRS Library can then be used within the ROS wrapper by instantiating the corresponding sensor module class and the definition of corresponding callbacks.

The setup of new, dedicated ROS nodes and the implementation of new sensor modules to the MaRS Library is described in the [Tutorial](#Tutorial) section, and by the [MaRS library](https://github.com/aau-cns/mars_lib) itself.

The MaRS ROS wrapper uses the MaRS library as a submodule. After cloning the `mars_ros` node, no additional download is needed (except of initializing the submodule).

**Austrian Patent Application Pending**

## Features

### ROS Wrapper

- Ready to use ROS nodes for common setups (Position, Pose, and GNSS sensor with IMU)
- Predefined sensor update modules (Plug and Play)
- Predefined RQT views
- Docker test environment

### The MaRS Framework

- Truly-Modular decoupling of Sensor-States from the essential Navigation-States
- Generalized covariance segmentation for Plug and Play state and covariance blocks
- Minimal State-Representation at any point in time
- Integration and removal of sensor modules during runtime
- Out of sequence sensor measurement handling
- Developed for computationally constrained platforms
- Efficient handling of asynchronous and multi-rate sensor information
- Separation between simple user interaction and the complexity of information handling

# Getting Started

## Setup and Building the Project

```sh
# Generate a catkin workspace (optional)
$ mkdir -p catkin_ws/src
$ cd catkin_ws
$ catkin init
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ cd src

# Get the mars_ros package
$ git clone https://github.com/aau-cns/mars_ros.git mars_ros
$ cd mars_ros
$ git submodule update --init --recursive

# Build the project and run tests
$ cd ../../
$ catkin build
```

## Testing Build with Docker

We auto-build a test environment for the MaRS ROS wrapper which can be used for testing your implementations

```bash
# The following runs the container, maps the source code (read only)
# and executes the script in 'docker/docker_application_test.sh'
$ docker run -it --rm \
  --network=host \
  -v "$(pwd)":/source:ro \
  gitlab.aau.at:5050/aau-cns-docker/docker_registry/mars_ros_test_env:latest
```

## Isolated Build with Docker

```sh
$ cd mars_ros # Enter the source directory
$ docker build --network=host -t mars_ros_test_env:latest . # Build the Docker image

# The following runs the container, maps the source code (read only)
# and executes the script in 'docker/docker_application_test.sh'
$ docker run -it --rm \
  --network=host \
  -v "$(pwd)":/source:ro \
  mars_ros_test_env:latest
```

# Usage

## Modules

| Module Name   | Description                                 |
| ------------- | ------------------------------------------- |
| position_node | An example 3DoF position update node        |
| pose_node     | An example 6DoF pose update node            |
| gps_node      | An example 3DoF GNSS coordinate update node |

## Parameter

The MaRS nodes define a set of standard parameters which are useful in paxis. The current set of parameters is stated below:

| Parameter                   | Description                                                  | Default Value |
| --------------------------- | ------------------------------------------------------------ | ------------- |
| `pub_on_prop`               | Turn on the publication of the core state for each propagation step | true          |
| `use_ros_time_now`          | Use ros time now for incomming measurements                  | false         |
| `verbose`                   | Extended CMD output                                          | false         |
| `verbose_out_of_order`      | Extended CMD output regarding out of order updates           | true          |
| `discard_ooo_prop_meas`     | Do not process out of order propagation sensor measurements (Out of order "Update Sensor" measurements are still handled) | false         |
| `pub_cov`                   | Publish the covariance of the core state with the ROS message | true          |
| `buffer_size`               | Size of the internal MaRS buffer                             | 2000          |
| `use_tcpnodelay`            | Use `tcpnodelay` for TransportHints                          | true          |
| `pub_cb_buffer_size`        | Max queue size for all outgoing messages                     | 1             |
| `sub_imu_cb_buffer_size`    | Max queue size for incoming IMU messages messages            | 200           |
| `sub_sensor_cb_buffer_size` | Max queue size for incoming update sensor  messages          | 1             |
| `respawn`                   | Define if the node restarts after exit                       | false         |

## Service

| Topic          | Type              | Description                     |
| -------------- | ----------------- | ------------------------------- |
| `init_service` | std_srvs::SetBool | Reset and initialize the filter |

## Topics

Because applications might use different message types, the example ROS nodes provide different subscribers to feed position or pose information. Each topic essentially feeds to the same update routine after converting the ROS message to an internal MaRS measurement type. **Note:** Only use one of the input messages for the example nodes to prevent confusion. If you want to use multiple sensors, please follow the [Tutorial](# Tutorial).

| Topic                          | Publisher / Subscriber | Type                                     | Content                                    |
| ------------------------------ | ---------------------- | ---------------------------------------- | ------------------------------------------ |
| `imu_in_topic`                 | subscriber             | sensor_msgs::Imu                         | Incoming IMU messages for propagation      |
| `transform_in_topic`           | subscriber             | geometry_msgs::TransformStamped          | Incoming 6Dof transform measurements       |
| `pose_in_topic`                | subscriber             | geometry_msgs::PoseStamped               | Incoming 6Dof Pose measurements            |
| `pose_with_cov_in_topic`       | subscriber             | geometry_msgs::PoseWithCovarianceStamped | Incoming 6Dof Pose w. cov measurements     |
| `odom_in_topic`                | subscriber             | nav_msgs::Odometry                       | Incoming 6Dof Odometry (Pose) measurements |
| `full_state_out_topic`         | publisher              | mars_ros::ExtCoreState                   |                                            |
| `full_state_lite_out_topic`    | publisher              | mars_ros::ExtCoreStateLite               |                                            |
| `pose_state_out_topic`         | publisher              | geometry_msgs::PoseStamped               |                                            |
| `<sensor>_cal_state_out_topic` | publisher              | geometry_msgs::PoseStamped               |                                            |

# Custom MaRS message types

## Default Navigation State

The Navigation-States (core states) are the essential states to localize a robot in the world, given the fact that these states are propagated with IMU measurements.

## General Nomenclature

The translation ![](https://latex.codecogs.com/svg.latex?_{A}\text{\textbf{P}}_{BC}) defines frame ![](https://latex.codecogs.com/svg.latex?C) with respect to frame ![](https://latex.codecogs.com/svg.latex?B) expressed in frame ![](https://latex.codecogs.com/svg.latex?A). The translation is expressed in frame ![](https://latex.codecogs.com/svg.latex?B) if the subscript ![](https://latex.codecogs.com/svg.latex?A) is not defined. The quaternion ![](https://latex.codecogs.com/svg.latex?\text{\textbf{q}}_{AB}) describes the rotation of frame ![](https://latex.codecogs.com/svg.latex?B) with respect to frame ![](https://latex.codecogs.com/svg.latex?A). ![](https://latex.codecogs.com/svg.latex?\text{\textbf{R}}_{(\text{\textbf{q}}_{AB})}\equiv\text{\textbf{R}}_{\text{\textbf{AB}}}) denotes the conversion of quaternion ![](https://latex.codecogs.com/svg.latex?\text{\textbf{q}}_{AB}) to its corresponding rotation matrix. Please note that this framework uses the Hamilton notation for the Quaternion representation.

### Symbols

| Symbol                                                       | Message name | Description                                                  |
| ------------------------------------------------------------ | ------------ | ------------------------------------------------------------ |
| ![](https://latex.codecogs.com/svg.latex?\text{\textbf{p}}_{WI}) | p_wi         | Translation of the robot IMU/body frame expressed w.r.t. the world frame |
| ![](https://latex.codecogs.com/svg.latex?\text{\textbf{v}}_{WI}) | v_wi         | Velocity of the robot IMU/body frame expressed w.r.t. the world frame |
| ![](https://latex.codecogs.com/svg.latex?\text{\textbf{q}}_{WI}) | q_wi         | Orientation of the robot IMU/body frame expressed w.r.t. the world frame (Hamiltonian) |
| ![](https://latex.codecogs.com/svg.latex?\text{\textbf{b}}_{\omega}) | b_w          | Gyroscopic bias                                              |
| ![](https://latex.codecogs.com/svg.latex?\text{\textbf{b}}_{a}) | b_a          | Accelerometer bias                                           |
| ![](https://latex.codecogs.com/svg.latex?\text{\textbf{P})   | cov          | Navigation state covariance                                  |

The fields `FRAME_TYPE`and `QUATERNION_TYPE` are used for consecutive processing. The states of MaRS are expressed in `ENU` and Quaternions are Hamiltonian.

## mars_ros::ExtCoreState

The `ExtCoreState` message contains the full set of information concerning the core navigation state. Definition of the message:

```
std_msgs/Header header
geometry_msgs/Vector3 p_wi
geometry_msgs/Vector3 v_wi
geometry_msgs/Quaternion q_wi
geometry_msgs/Vector3 b_w
geometry_msgs/Vector3 b_a
float64[225] cov
uint8 FRAME_TYPE
uint8 QUATERNION_TYPE
```

## mars_ros::ExtCoreStateLite

The `ExtCoreStateLite` message contains only the essential state components for navigation. This message can be used if the bandwidth to e.g. a flight controller, is limited. Definition of the message:

```
std_msgs/Header header
geometry_msgs/Vector3 p_wi
geometry_msgs/Vector3 v_wi
geometry_msgs/Quaternion q_wi

uint8 FRAME_TYPE
uint8 QUATERNION_TYPE
```

# Programming

The codebase is mostly C++ based and follows the C++ Google style convention. A C-Lang file with formatting definitions / for auto formatting can be found in the root directory of the project `mars_ros/.clang-format`.

# Tutorial

As stated in the introduction, the ROS MaRS wrapper does not define the general sensor modules. Sensor modules are defined (ROS independent) by the MaRS Library. The usage of sensors within the ROS wrapper is done through the MaRS Library API. The definition and design of a new ROS node/middleware integration is described by the [MaRS Lib Integration Tutorial](https://github.com/aau-cns/mars_lib/blob/main/README.md#stand-alone-usage-and-middleware-integration).

# Project Layout

## Package Layout/Codebase

Generated with `tree -a -L 3 --noreport --charset unicode > layout.md`

```
.
|-- cfg
|   `-- mars.cfg
|-- .clang-format
|-- cmake
|   `-- mars_libConfig.cmake
|-- CMakeLists.txt
|-- docker
|   |-- docker_application_test.sh
|   `-- dockerfile
|-- .gitmodules
|-- include
|   |-- mars_msg_conv.h
|   |-- mars_wrapper_gps.h
|   |-- mars_wrapper_pose.h
|   `-- mars_wrapper_position.h
|-- launch
|   |-- config
|   |   |-- gps_config.yaml
|   |   |-- pose_config.yaml
|   |   `-- position_config.yaml
|   |-- mars_gps.launch
|   |-- mars_gps_template.launch
|   |-- mars_pose.launch
|   |-- mars_pose_template.launch
|   |-- mars_position.launch
|   |-- px4_sim.launch
|   `-- rqt_pose.launch
|-- LICENSE
|-- mars_lib
|       `-- <submodule>
|-- msg
|   |-- ExtCoreStateLite.msg
|   `-- ExtCoreState.msg
|-- package.xml
|-- README.md
|-- resources
|   |-- a-astro-space-font.zip
|   |-- cov_segmentation.png
|   |-- cov_segmentation.svg
|   |-- mars_ros_logo.png
|   `-- mars_ros_logo.svg
|-- rosdoc.yaml
|-- rqt_perspective
|   `-- mars_pose.perspective
`-- src
    |-- mars_node.cpp
    |-- mars_wrapper_gps.cpp
    |-- mars_wrapper_pose.cpp
    `-- mars_wrapper_position.cpp
```

# Contact

For further information, please contact [Christian Brommer](mailto:christian.brommer@aau.at)

# License

This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the [LICENSE](./LICENSE) file. No license in patents is granted.

## Usage for academic purposes

If you use this software in an academic research setting, please cite the corresponding paper and consult the [LICENSE](./LICENSE) file for a detailed explanation.

```latex
@inproceedings{brommer2020,
   author   = {Brommer, Christian and Jung, Roland and Steinbrener, Jan and Weiss, Stephan},
   doi      = {10.1109/LRA.2020.3043195},
   journal  = {IEEE Robotics and Automation Letters},
   title    = {{MaRS : A Modular and Robust Sensor-Fusion Framework}},
   year     = {2020}
}
