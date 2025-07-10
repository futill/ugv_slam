#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std::placeholders;

class SensorSyncNode : public rclcpp::Node
{
public:
  SensorSyncNode() : Node("sensor_sync_node")
  {
    // 初始化订阅者
    laser_sub_.subscribe(this, "/scan");  // 假设激光雷达话题为 /scan
    odom_sub_.subscribe(this, "/odom");

    // 配置 ApproximateTimeSynchronizer
    // queue_size=10: 消息队列大小
    // slop=0.1: 时间窗口为 0.1 秒
    sync_ = std::make_shared<Sync>(
        10, laser_sub_, odom_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
    sync_->registerCallback(std::bind(&SensorSyncNode::callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Sensor sync node initialized");
  }

private:
  // 定义同步策略
  using Sync = message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::LaserScan,
          nav_msgs::msg::Odometry>>;
  using LaserScanMsg = sensor_msgs::msg::LaserScan;
  using OdometryMsg = nav_msgs::msg::Odometry;

  // 订阅者
  message_filters::Subscriber<LaserScanMsg> laser_sub_;
  message_filters::Subscriber<OdometryMsg> odom_sub_;
  // 同步器
  std::shared_ptr<Sync> sync_;

  // 回调函数，处理同步后的消息
  void callback(const LaserScanMsg::ConstSharedPtr& laser_msg,
                const OdometryMsg::ConstSharedPtr& odom_msg)
  {
    // 获取时间戳
    auto laser_stamp = laser_msg->header.stamp;
    auto odom_stamp = odom_msg->header.stamp;

    // 使用正确的格式说明符：%d 用于 int，%u 用于 unsigned int
    // RCLCPP_INFO(this->get_logger(),
    //             "Synced: LaserScan at %d.%09u, Odometry at %d.%09u",
    //             laser_stamp.sec, laser_stamp.nanosec,
    //             odom_stamp.sec, odom_stamp.nanosec);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}