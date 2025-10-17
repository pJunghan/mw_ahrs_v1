// Copyright (c) 2023, ROAS Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MW_AHRS_ROS2__MW_AHRS_DRIVER_HPP_
#define MW_AHRS_ROS2__MW_AHRS_DRIVER_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "realtime_tools/realtime_publisher.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"

class MwAhrsDriver : public rclcpp::Node
{
public:
  MwAhrsDriver(const std::string& node);

  virtual ~MwAhrsDriver();

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Read message
   */
  void read();

  /**
   * \brief Start streaming IMU sensor data
   */
  void start();

  /**
   * \brief Reset IMU
   * \param req Request
   * \param resp Response
   */
  void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  /**
   * \brief Publish IMU sensor data
   */
  void publishData();

  /// For serial communication
  std::shared_ptr<LibSerial::SerialPort> serial_;

private:
  /// ROS2 parameters
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> rp_imu_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> rp_rpy_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>> rp_mag_;

  // Timer for publisher
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Thread for reading message
  std::shared_ptr<std::thread> read_thread_;

  /// Serial parameter
  std::string port_;

  /// Frame ID
  std::string frame_id_;

  /// Sensor version
  std::string version_;
};

#endif  // MW_AHRS_ROS2__MW_AHRS_DRIVER_HPP_