#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

class TimeSyncNode : public rclcpp::Node {
public:
    TimeSyncNode() : Node("time_sync_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
        this->declare_parameter("slop", 0.2);  // 时间窗口 0.2 秒
        this->declare_parameter("max_time_diff_log", 0.1);  // 记录 >0.1 秒差异
        this->declare_parameter("diag_publish_rate", 1.0);  // 诊断频率 1 Hz
        this->declare_parameter("publish_rate", 10.0);  // 发布频率 10 Hz

        // 订阅器初始化
        lidar_sub_.subscribe(this, "/lsn10/scan");
        odom_sub_.subscribe(this, "/vio/odom");  // 这里订阅 Odometry

        // 同步策略定义（LaserScan 和 Odometry）
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::LaserScan,
            nav_msgs::msg::Odometry
        >;

        int queue_size = 20;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue_size), lidar_sub_, odom_sub_
        );

        // 注册同步回调
        sync_->registerCallback(std::bind(
            &TimeSyncNode::syncCallback, this,
            std::placeholders::_1, std::placeholders::_2
        ));

        // 诊断信息发布器
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::KeyValue>("/time_sync/diagnostics", 10);

        // 同步后消息发布器
        synced_lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        synced_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        RCLCPP_INFO(this->get_logger(),
            "时间同步节点启动, 时间窗口: %.3f秒, 发布频率: %.1f Hz",
            this->get_parameter("slop").as_double(),
            this->get_parameter("publish_rate").as_double());
    }

private:
    // 订阅器
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;

    // 同步器
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::LaserScan,
            nav_msgs::msg::Odometry
        >
    >> sync_;

    // 发布器
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr diag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr synced_lidar_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr synced_odom_pub_;

    void syncCallback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
    {
        auto scan_time = rclcpp::Time(scan_msg->header.stamp);
        auto odom_time = rclcpp::Time(odom_msg->header.stamp);

        static rclcpp::Time last_scan_time(0, 0, RCL_ROS_TIME);
        if (scan_time < last_scan_time) {
            RCLCPP_WARN(this->get_logger(), "丢弃时间戳倒退的 LaserScan 消息: %.3f < %.3f",
                        scan_time.seconds(), last_scan_time.seconds());
            return;
        }
        last_scan_time = scan_time;

        double time_diff_odom = (scan_time - odom_time).seconds();

        // 发布诊断信息
        double diag_rate = this->get_parameter("diag_publish_rate").as_double();
        static rclcpp::Time last_diag_time(0, 0, RCL_ROS_TIME);
        if ((scan_time - last_diag_time).seconds() >= 1.0 / diag_rate) {
            diagnostic_msgs::msg::KeyValue diag_msg;
            diag_msg.key = "time_diff_odom";
            diag_msg.value = std::to_string(time_diff_odom);
            diag_pub_->publish(diag_msg);
            last_diag_time = scan_time;
        }

        // 记录过大时间差
        double max_diff = this->get_parameter("max_time_diff_log").as_double();
        if (std::abs(time_diff_odom) > max_diff) {
            RCLCPP_WARN(this->get_logger(),
                       "大时间差警告 (Odometry): 激光@%.3f, 里程计@%.3f, 差异=%.4f秒",
                       scan_time.seconds(), odom_time.seconds(), time_diff_odom);
        }

        // 发布同步后的消息
        synced_lidar_pub_->publish(*scan_msg);
        synced_odom_pub_->publish(*odom_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "已同步并转发消息, 时间差: 里程计=%.4f秒",
                     time_diff_odom);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TimeSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
