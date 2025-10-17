// SPDX-License-Identifier: Apache-2.0
// imu_orientation_tf_node.cpp
//
// /imu/data (sensor_msgs/Imu)의 orientation을 사용해
// parent_frame → child_frame TF를 계속 브로드캐스트합니다.
// 기본값: parent_frame=imu_link, child_frame=imu_visual

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ImuOrientationTfNode : public rclcpp::Node
{
public:
  ImuOrientationTfNode() : rclcpp::Node("imu_orientation_tf")
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "imu_link");
    child_frame_  = this->declare_parameter<std::string>("child_frame",  "imu_visual");

    // TF 브로드캐스터
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // IMU 구독 (sensor_data QoS와 유사)
    rclcpp::QoS qos(10);
    qos.best_effort();
    qos.keep_last(10);
    qos.durability_volatile();

    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos,
      std::bind(&ImuOrientationTfNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Publishing dynamic TF: %s -> %s (from /imu/data orientation)",
      parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // orientation 유효성 간단 체크(모두 0이고 w=0이면 무시)
    if (msg->orientation.x == 0.0 && msg->orientation.y == 0.0 &&
        msg->orientation.z == 0.0 && msg->orientation.w == 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "IMU orientation is zero; skipping TF publish");
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    // 타임스탬프: 메시지 헤더가 0이면 현재 시간을 사용(동기화 용이)
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
      t.header.stamp = this->now();
    else
      t.header.stamp = msg->header.stamp;

    t.header.frame_id = parent_frame_;
    t.child_frame_id  = child_frame_;

    // 위치는 0 (회전만 반영)
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    // IMU orientation 그대로 사용
    t.transform.rotation = msg->orientation;

    tf_br_->sendTransform(t);
  }

  std::string parent_frame_;
  std::string child_frame_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOrientationTfNode>());
  rclcpp::shutdown();
  return 0;
}
