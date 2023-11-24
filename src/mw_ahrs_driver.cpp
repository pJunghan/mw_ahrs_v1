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

#include "mw_ahrs_ros2/mw_ahrs_driver.hpp"

const std::string eol("\n");
const std::size_t max_line_length(128);

using namespace std::literals;

MwAhrsDriver::MwAhrsDriver(const std::string& node) : Node(node)
{
  port_ = this->declare_parameter("port", "/dev/ttyUSB0");
  frame_id_ = this->declare_parameter("frame_id", "imu_link");
  version_ = this->declare_parameter("version", "v2");

  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
      "imu/reset", std::bind(&MwAhrsDriver::reset, this, std::placeholders::_1, std::placeholders::_2));
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  pub_rpy_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 1);
  pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
}

MwAhrsDriver::~MwAhrsDriver()
{
  serial_->close();
}

bool MwAhrsDriver::init()
{
  rp_imu_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(pub_imu_);
  rp_imu_->msg_.header.frame_id = frame_id_;
  rp_imu_->msg_.orientation_covariance = { 0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025 };
  rp_imu_->msg_.angular_velocity_covariance = { 0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02 };
  rp_imu_->msg_.linear_acceleration_covariance = { 0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04 };

  rp_rpy_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(pub_rpy_);
  rp_rpy_->msg_.header.frame_id = frame_id_;

  rp_mag_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>>(pub_mag_);
  rp_mag_->msg_.header.frame_id = frame_id_;

  // Initial setting for serial communication
  serial_ = std::make_shared<serial::Serial>();
  serial_->setPort(port_);
  serial_->setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  serial_->setTimeout(to);

  try
  {
    serial_->open();
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "serial::IOException: " << e.what());
  }

  // Check the serial port
  if (serial_->isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "MW AHRS driver cocbnnected to %s at %i baud", port_.c_str(), 115200);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "MW AHRS driver failed to connect to %s", port_.c_str());
    return false;
  }

  start();

  publish_timer_ = this->create_wall_timer(10ms, std::bind(&MwAhrsDriver::publishData, this));
  read_thread_ = std::make_shared<std::thread>(&MwAhrsDriver::read, this);

  return true;
}

void MwAhrsDriver::read()
{
  while (rclcpp::ok())
  {
    if (serial_->available())
    {
      std::string msg = serial_->readline(max_line_length, eol);
      // RCLCPP_INFO_STREAM(this->get_logger(), "Message : " << msg);

      int cnt = 0;
      std::string data[1000];
      char* buff = new char[1000];
      strcpy(buff, msg.c_str());
      char* tok = strtok(buff, " ");

      while (tok != nullptr)
      {
        data[cnt++] = std::string(tok);
        tok = strtok(nullptr, " ");
      }

      int num = 0;
      if (version_ == "v1")
        num = 12;
      else if (version_ == "v2")
        num = 14;

      if (cnt == num)
      {
        rp_rpy_->msg_.vector.x = stod(data[6]) * M_PI / 180.0;
        rp_rpy_->msg_.vector.y = stod(data[7]) * M_PI / 180.0;
        rp_rpy_->msg_.vector.z = stod(data[8]) * M_PI / 180.0;

        rp_imu_->msg_.linear_acceleration.x = stod(data[0]) * 9.80665;
        rp_imu_->msg_.linear_acceleration.y = stod(data[1]) * 9.80665;
        rp_imu_->msg_.linear_acceleration.z = stod(data[2]) * 9.80665;

        rp_imu_->msg_.angular_velocity.x = stod(data[3]) * M_PI / 180.0;
        rp_imu_->msg_.angular_velocity.y = stod(data[4]) * M_PI / 180.0;
        rp_imu_->msg_.angular_velocity.z = stod(data[5]) * M_PI / 180.0;

        tf2::Quaternion quat;
        quat.setRPY(rp_rpy_->msg_.vector.x, rp_rpy_->msg_.vector.y, rp_rpy_->msg_.vector.z);

        rp_imu_->msg_.orientation.x = quat[0];
        rp_imu_->msg_.orientation.y = quat[1];
        rp_imu_->msg_.orientation.z = quat[2];
        rp_imu_->msg_.orientation.w = quat[3];

        rp_mag_->msg_.magnetic_field.x = stod(data[9]) * 0.000001;
        rp_mag_->msg_.magnetic_field.y = stod(data[10]) * 0.000001;
        rp_mag_->msg_.magnetic_field.z = stod(data[11]) * 0.000001;
      }
    }
  }
}

void MwAhrsDriver::start()
{
  std::string start = "ss=15\n";
  serial_->write(start);

  std::string frequency = "sp=10\n";
  serial_->write(frequency);
}

void MwAhrsDriver::reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  std::string msg = "rst\n";
  serial_->write(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  start();
  resp->success = true;
}

void MwAhrsDriver::publishData()
{
  if (rp_rpy_->trylock())
  {
    rp_rpy_->msg_.header.stamp = this->now();
    rp_rpy_->unlockAndPublish();
  }

  if (rp_imu_->trylock())
  {
    rp_imu_->msg_.header.stamp = this->now();
    rp_imu_->unlockAndPublish();
  }

  if (rp_mag_->trylock())
  {
    rp_mag_->msg_.header.stamp = this->now();
    rp_mag_->unlockAndPublish();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MwAhrsDriver>("mw_ahrs_driver_node");

  if (node->init())
  {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
  }

  rclcpp::shutdown();
  return 0;
}