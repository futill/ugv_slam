#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

class CmdVelToSerial : public rclcpp::Node
{
public:
    CmdVelToSerial() : Node("cmd_vel_to_serial")
    {
        // 声明并获取串口参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

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
        // 构建 JSON 数据
        json j;
        j["T"] = 13;
        j["X"] = msg->linear.x;  // 线速度 (m/s)
        j["Z"] = msg->angular.z; // 角速度 (rad/s)

        // 序列化 JSON 并添加换行符
        std::string json_str = j.dump() + "\n";

        // 发送到串口
        try
        {
            serial_.write(json_str);
            RCLCPP_INFO(this->get_logger(), "发送到串口: %s", json_str.c_str());
        }
        catch (const serial::SerialException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "串口写入失败: %s", e.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    serial::Serial serial_;
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