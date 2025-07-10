#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <string>
#include <vector>
#include <cmath>

class CmdVelToSerial : public rclcpp::Node
{
public:
    CmdVelToSerial() : Node("cmd_vel_to_serial")
    {
        // 声明并获取参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyS1");
        this->declare_parameter<int>("baud_rate", 9600);
        this->declare_parameter<double>("wheel_base", 0.06);    // 轴距，单位：m
        this->declare_parameter<double>("wheel_radius", 0.03);  // 轮子半径，单位：m
        this->declare_parameter<double>("max_rpm", 100.0);    // 电机最大 RPM

        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // 计算速度单位转换因子
        speed_scale_ = (65535.0 * 60.0) / (2.0 * M_PI);

        // 初始化串口
        try
        {
            serial_.setPort(serial_port);
            serial_.setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "串口 %s 已打开，波特率 %d", serial_port.c_str(), baud_rate);
        }
        catch (const serial::SerialException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s: %s", serial_port.c_str(), e.what());
            throw;
        }

        // 订阅 /cmd_vel 话题
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelToSerial::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 计算左右轮线速度
        double v = msg->linear.x;  // 线速度 (m/s)
        double w = msg->angular.z; // 角速度 (rad/s)
        double v_left = v - (w * wheel_base_ / 2.0);
        double v_right = v + (w * wheel_base_ / 2.0);

        // 转换为轮子角速度 (rad/s)
        double omega_left = v_left / wheel_radius_;
        double omega_right = v_right / wheel_radius_;

        // 转换为 RPM
        double rpm_left = omega_left * 60.0 / (2.0 * M_PI);
        double rpm_right = omega_right * 60.0 / (2.0 * M_PI);

        // 转换为硬件速度值
        int16_t speed_left = static_cast<int16_t>(std::abs(rpm_left) );
        int16_t speed_right = static_cast<int16_t>(std::abs(rpm_right));


        // 确定方向（0: 正向, 1: 反向）
        uint8_t dir_left = (rpm_left >= 0) ? 0 : 1;
        uint8_t dir_right = (rpm_right >= 0) ? 1 : 0;

        // 构造二进制消息
        std::vector<uint8_t> buffer = {
            0xA5, // 起始字节
            0xA6, // 命令字节
            dir_left, // 电机1方向
            dir_right, // 电机2方向
            static_cast<uint8_t>(speed_left >> 8),  // 电机1速度高字节
            static_cast<uint8_t>(speed_left & 0xFF), // 电机1速度低字节
            static_cast<uint8_t>(speed_right >> 8),  // 电机2速度高字节
            static_cast<uint8_t>(speed_right & 0xFF), // 电机2速度低字节
            0x6B, // 结束字节
            0x5B
        };

        // 发送到串口
        try
        {
            serial_.write(buffer);
            RCLCPP_INFO(this->get_logger(), "发送到串口: A5 A6 %02X %02X %04X %04X 6B",
                        dir_left, dir_right, speed_left, speed_right);
        }
        catch (const serial::SerialException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "串口写入失败: %s", e.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    serial::Serial serial_;
    double wheel_base_;   // 轴距
    double wheel_radius_; // 轮子半径
    double max_rpm_;      // 最大 RPM
    double speed_scale_;  // 速度单位转换因子
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<CmdVelToSerial>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "节点启动失败: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}