#include <memory>
#include <chrono>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class TfPosePublisher : public rclcpp::Node
{
public:
  TfPosePublisher()
  : Node("tf_pose_publisher")
  {
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose_data", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&TfPosePublisher::timer_callback, this));
  }

private:
  // 自定义函数：从四元数计算Yaw角
  double getYawFromQuaternion(const tf2::Quaternion & q) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      // 等待并获取map到base_link的变换
      transform_stamped = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    double x = transform_stamped.transform.translation.x;
    double y = transform_stamped.transform.translation.y;
    double z = transform_stamped.transform.translation.z;

    tf2::Quaternion q(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);

    double yaw = getYawFromQuaternion(q);

    auto pose_msg = geometry_msgs::msg::Pose2D();
    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.theta = yaw;

    pose_pub_->publish(pose_msg);

    RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f, z: %.3f, yaw: %.3f", x, y, z, yaw);
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
