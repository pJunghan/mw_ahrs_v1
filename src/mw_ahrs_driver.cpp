// SPDX-License-Identifier: Apache-2.0
// Author: Brighten Lee + 개선: ChatGPT
//
// src/mw_ahrs_driver_node.cpp
//
// 빌드 시 필요 라이브러리: libserial-dev
// sudo apt install libserial-dev

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <realtime_tools/realtime_publisher.h>
#include <libserial/SerialPort.h>

#include <thread>
#include <atomic>
#include <sstream>
#include <vector>
#include <chrono>
#include <mutex>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class MwAhrsDriver : public rclcpp::Node
{
public:
  explicit MwAhrsDriver(const std::string& node_name)
  : Node(node_name)
  {
    // ---- Parameters ----
    port_          = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    frame_id_      = this->declare_parameter<std::string>("frame_id", "imu_link");
    version_param_ = this->declare_parameter<std::string>("version", "");
    baud_rate_     = this->declare_parameter<int>("baud_rate", 115200);
    newline_crlf_  = this->declare_parameter<bool>("newline_crlf", true);
    start_stream_  = this->declare_parameter<bool>("start_stream", true);
    start_ss_      = this->declare_parameter<int>("start_ss", 15);
    start_sp_      = this->declare_parameter<int>("start_sp", 10);
    log_raw_       = this->declare_parameter<bool>("log_raw", false);
    use_rt_pub_    = this->declare_parameter<bool>("use_realtime_pub", true);

    // ---- Publishers ----
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    pub_rpy_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 10);
    pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

    if (use_rt_pub_) {
      rp_imu_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(pub_imu_);
      rp_rpy_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(pub_rpy_);
      rp_mag_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>>(pub_mag_);

      rp_imu_->msg_.header.frame_id = frame_id_;
      rp_imu_->msg_.orientation_covariance        = {0.0025,0,0,0,0.0025,0,0,0,0.0025};
      rp_imu_->msg_.angular_velocity_covariance   = {0.02,0,0,0,0.02,0,0,0,0.02};
      rp_imu_->msg_.linear_acceleration_covariance= {0.04,0,0,0,0.04,0,0,0,0.04};
      rp_rpy_->msg_.header.frame_id = frame_id_;
      rp_mag_->msg_.header.frame_id = frame_id_;
    }

    srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
      "imu/reset", std::bind(&MwAhrsDriver::onReset, this, _1, _2));

    serial_ = std::make_shared<LibSerial::SerialPort>();

    publish_timer_ = this->create_wall_timer(10ms, std::bind(&MwAhrsDriver::onPublishTimer, this));

    if (!openSerialAndStart()) {
      RCLCPP_ERROR(this->get_logger(), "Initialization failed. Check port/permissions/baud.");
    } else {
      read_thread_ = std::thread(&MwAhrsDriver::readLoop, this);
    }
  }

  ~MwAhrsDriver() override
  {
    running_.store(false);
    if (read_thread_.joinable()) read_thread_.join();
    if (serial_ && serial_->IsOpen()) serial_->Close();
  }

private:
  std::string port_, frame_id_, version_param_;
  int baud_rate_, start_ss_, start_sp_;
  bool newline_crlf_, start_stream_, log_raw_, use_rt_pub_;
  int expected_tokens_{0};

  std::shared_ptr<LibSerial::SerialPort> serial_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> rp_imu_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> rp_rpy_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>> rp_mag_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  bool openSerialAndStart()
  {
    try { serial_->Open(port_); }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "LibSerial::OpenFailed: %s", e.what());
      return false;
    }

    serial_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_->SetParity(LibSerial::Parity::PARITY_NONE);
    serial_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_->FlushIOBuffers();
    std::this_thread::sleep_for(100ms);

    if (serial_->IsOpen())
      RCLCPP_INFO(this->get_logger(), "MW AHRS connected to %s at %d baud", port_.c_str(), baud_rate_);
    else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s", port_.c_str());
      return false;
    }

    if (start_stream_) sendStartCommands();
    running_.store(true);
    return true;
  }

  void sendStartCommands()
  {
    const std::string nl = newline_crlf_ ? "\r\n" : "\n";
    try {
      serial_->Write("ss=" + std::to_string(start_ss_) + nl);
      std::this_thread::sleep_for(20ms);
      serial_->Write("sp=" + std::to_string(start_sp_) + nl);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to send start commands: %s", e.what());
    }
  }

  void onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    const std::string nl = newline_crlf_ ? "\r\n" : "\n";
    serial_->Write("rst" + nl);
    std::this_thread::sleep_for(1s);
    if (start_stream_) sendStartCommands();
    resp->success = true;
    resp->message = "IMU reset and stream restarted";
  }

  void onPublishTimer() {}

  // -------------------- readLoop --------------------
  void readLoop()
  {
    const char eol = '\n';
    const size_t timeout_ms = 300;

    while (rclcpp::ok() && running_.load()) {
      try {
        if (!serial_->IsOpen()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Serial not open, retrying...");
          std::this_thread::sleep_for(200ms);
          continue;
        }

        if (!serial_->IsDataAvailable()) {
          std::this_thread::sleep_for(2ms);
          continue;
        }

        std::string line;
        serial_->ReadLine(line, eol, timeout_ms);
        if (line.empty()) continue;

        if (log_raw_)
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "RAW: %s", line.c_str());

        // ---- 구분자 통일 ----
        for (char& c : line)
          if (c == ',' || c == '\t') c = ' ';
        if (!line.empty() && line.back() == '\r') line.pop_back();

        std::istringstream iss(line);
        std::vector<std::string> tokens;
        for (std::string t; iss >> t; ) tokens.push_back(t);

        // ---- 기대 토큰 수 자동판별 (12/14/16 허용) ----
        int expected = expected_tokens_;
        if (!version_param_.empty()) {
          if (version_param_ == "v1") expected = 14;
          else if (version_param_ == "v2") expected = 16;
          else if (version_param_ == "v12") expected = 12;
          else RCLCPP_WARN(this->get_logger(), "Unknown version param: %s", version_param_.c_str());
        } else if (expected == 0) {
          const int n = tokens.size();
          if (n == 12 || n == 14 || n == 16) {
            expected = n;
            expected_tokens_ = expected;
            RCLCPP_INFO(this->get_logger(), "Detected token format: %d", expected);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Unexpected token count=%zu, line='%s'", tokens.size(), line.c_str());
            continue;
          }
        }

        if ((int)tokens.size() != expected) continue;

        auto to_d = [](const std::string& s)->double { return std::stod(s); };

        // ---- 필드 파싱 (앞의 12개만 사용) ----
        const double ax = to_d(tokens[0]) * 9.80665;
        const double ay = to_d(tokens[1]) * 9.80665;
        const double az = to_d(tokens[2]) * 9.80665;
        const double gx = to_d(tokens[3]) * M_PI / 180.0;
        const double gy = to_d(tokens[4]) * M_PI / 180.0;
        const double gz = to_d(tokens[5]) * M_PI / 180.0;
        const double roll  = to_d(tokens[6]) * M_PI / 180.0;
        const double pitch = to_d(tokens[7]) * M_PI / 180.0;
        const double yaw   = to_d(tokens[8]) * M_PI / 180.0;
        const double mx = to_d(tokens[9])  * 1e-6;
        const double my = to_d(tokens[10]) * 1e-6;
        const double mz = to_d(tokens[11]) * 1e-6;

        const rclcpp::Time now = this->now();

        // ---- /imu/rpy ----
        if (use_rt_pub_) {
          if (rp_rpy_->trylock()) {
            rp_rpy_->msg_.header.stamp = now;
            rp_rpy_->msg_.header.frame_id = frame_id_;
            rp_rpy_->msg_.vector.x = roll;
            rp_rpy_->msg_.vector.y = pitch;
            rp_rpy_->msg_.vector.z = yaw;
            rp_rpy_->unlockAndPublish();
          }
        } else {
          geometry_msgs::msg::Vector3Stamped msg;
          msg.header.stamp = now;
          msg.header.frame_id = frame_id_;
          msg.vector.x = roll;
          msg.vector.y = pitch;
          msg.vector.z = yaw;
          pub_rpy_->publish(msg);
        }

        // ---- /imu/data ----
        if (use_rt_pub_) {
          if (rp_imu_->trylock()) {
            rp_imu_->msg_.header.stamp = now;
            rp_imu_->msg_.header.frame_id = frame_id_;
            rp_imu_->msg_.linear_acceleration.x = ax;
            rp_imu_->msg_.linear_acceleration.y = ay;
            rp_imu_->msg_.linear_acceleration.z = az;
            rp_imu_->msg_.angular_velocity.x = gx;
            rp_imu_->msg_.angular_velocity.y = gy;
            rp_imu_->msg_.angular_velocity.z = gz;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            rp_imu_->msg_.orientation.x = q.x();
            rp_imu_->msg_.orientation.y = q.y();
            rp_imu_->msg_.orientation.z = q.z();
            rp_imu_->msg_.orientation.w = q.w();
            rp_imu_->unlockAndPublish();
          }
        } else {
          sensor_msgs::msg::Imu msg;
          msg.header.stamp = now;
          msg.header.frame_id = frame_id_;
          msg.linear_acceleration.x = ax;
          msg.linear_acceleration.y = ay;
          msg.linear_acceleration.z = az;
          msg.angular_velocity.x = gx;
          msg.angular_velocity.y = gy;
          msg.angular_velocity.z = gz;
          tf2::Quaternion q;
          q.setRPY(roll, pitch, yaw);
          msg.orientation.x = q.x();
          msg.orientation.y = q.y();
          msg.orientation.z = q.z();
          msg.orientation.w = q.w();
          pub_imu_->publish(msg);
        }

        // ---- /imu/mag ----
        if (use_rt_pub_) {
          if (rp_mag_->trylock()) {
            rp_mag_->msg_.header.stamp = now;
            rp_mag_->msg_.header.frame_id = frame_id_;
            rp_mag_->msg_.magnetic_field.x = mx;
            rp_mag_->msg_.magnetic_field.y = my;
            rp_mag_->msg_.magnetic_field.z = mz;
            rp_mag_->unlockAndPublish();
          }
        } else {
          sensor_msgs::msg::MagneticField msg;
          msg.header.stamp = now;
          msg.header.frame_id = frame_id_;
          msg.magnetic_field.x = mx;
          msg.magnetic_field.y = my;
          msg.magnetic_field.z = mz;
          pub_mag_->publish(msg);
        }

      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "readLoop exception: %s", e.what());
        std::this_thread::sleep_for(50ms);
      }
    }
  }
  // -------------------- end readLoop --------------------
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MwAhrsDriver>("mw_ahrs_driver_node");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
