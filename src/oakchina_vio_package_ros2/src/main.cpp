#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <signal.h>
#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>
#include <mutex>
#include <cstring>
#include <sstream>
#include "carina_a1088.h"
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

std::mutex image_mtx;
cv::Mat left_image;
cv::Mat right_image;
cv::Mat left_image1;
cv::Mat right_image1;
double image_ts = -1;

std::mutex imu_mtx;
std::vector<float> imu_data;
double imu_ts = -1;

std::mutex vsync_mtx;
double vsync_ts = -1;

std::mutex pose_mtx;
float pose_data[32] = {0,};
double pose_ts = -1;

std::mutex points_mtx;
carina_points points_data;
double points_ts = -1;

std::mutex event_mtx;
unsigned char event = 0;

std::atomic<bool> shutdown_requested(false);

void signal_handle(int signal) {
    if (SIGINT == signal) {
        shutdown_requested = true;
    }
}

bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

bool isValidRotationMatrix(const double* R) {
    // 检查是否包含 NaN 或 inf
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(R[i])) {
            return false;
        }
    }
    // 检查行列式是否接近 1
    double det = R[0] * (R[4] * R[8] - R[5] * R[7]) -
                 R[1] * (R[3] * R[8] - R[5] * R[6]) +
                 R[2] * (R[3] * R[7] - R[4] * R[6]);
    return std::abs(det - 1.0) < 1e-6;
}

tf2::Quaternion rotationMatrixToQuaternion(const double* R) {
    if (!isValidRotationMatrix(R)) {
        RCLCPP_WARN(rclcpp::get_logger("vio"), "Invalid rotation matrix, returning identity quaternion");
        return tf2::Quaternion(0.0, 0.0, 0.0, 1.0); // 返回单位四元数
    }
    tf2::Quaternion q;
    double trace = R[0] + R[4] + R[8];

    if (trace > 0) {
        double s = sqrt(trace + 1.0) * 2;
        q.setW(0.25 * s);
        q.setX((R[7] - R[5]) / s);
        q.setY((R[2] - R[6]) / s);
        q.setZ((R[3] - R[1]) / s);
    } else {
        if (R[0] > R[4] && R[0] > R[8]) {
            double s = sqrt(1.0 + R[0] - R[4] - R[8]) * 2;
            q.setW((R[7] - R[5]) / s);
            q.setX(0.25 * s);
            q.setY((R[1] + R[3]) / s);
            q.setZ((R[2] + R[6]) / s);
        } else if (R[4] > R[8]) {
            double s = sqrt(1.0 + R[4] - R[0] - R[8]) * 2;
            q.setW((R[2] - R[6]) / s);
            q.setX((R[1] + R[3]) / s);
            q.setY(0.25 * s);
            q.setZ((R[7] + R[5]) / s);
        } else {
            double s = sqrt(1.0 + R[8] - R[0] - R[4]) * 2;
            q.setW((R[3] - R[1]) / s);
            q.setX((R[2] + R[6]) / s);
            q.setY((R[7] + R[5]) / s);
            q.setZ(0.25 * s);
        }
    }
    q.normalize();
    return q;
}

void CarinaA1088PoseCallBack(float *pose, double ts) {
    std::lock_guard<std::mutex> auto_lock(pose_mtx);
    memcpy(pose_data, pose, sizeof(float) * 32);
    pose_ts = ts;
    RCLCPP_INFO(rclcpp::get_logger("vio"), "Pose callback: tx=%.3f, ty=%.3f, tz=%.3f, R00=%.3f, R01=%.3f, R02=%.3f",
                pose_data[12], pose_data[13], pose_data[14], pose_data[0], pose_data[4], pose_data[8]);
}

void CarinaA1088VsyncCallBack(double ts) {
    std::lock_guard<std::mutex> auto_lock(vsync_mtx);
    vsync_ts = ts;
}

void CarinaA1088ImuCallBack(float *imu, double ts) {
    std::lock_guard<std::mutex> auto_lock(imu_mtx);
    if (imu != nullptr) {
        imu_data.resize(6);
        for (int i = 0; i < 6; i++) {
            imu_data[i] = imu[i];
        }
    }
    imu_ts = ts;
}

void CarinaA1088CameraCallBack(char *left, char *right, char *left1, char *right1, double ts, int w, int h) {
    std::lock_guard<std::mutex> auto_lock(image_mtx);
    if (left != nullptr) {
        left_image = cv::Mat(h, w, CV_8UC1);
        memcpy(left_image.data, left, w * h);
    }
    if (right != nullptr) {
        right_image = cv::Mat(h, w, CV_8UC1);
        memcpy(right_image.data, right, w * h);
    }
    if (left1 != nullptr) {
        left_image1 = cv::Mat(h, w, CV_8UC1);
        memcpy(left_image1.data, left1, w * h);
    }
    if (right1 != nullptr) {
        right_image1 = cv::Mat(h, w, CV_8UC1);
        memcpy(right_image1.data, right1, w * h);
    }
    image_ts = ts;
}

void CarinaA1088PointsCallBack(carina_points &points, double ts) {
    std::lock_guard<std::mutex> auto_lock(points_mtx);
    for (int i = 0; i < points_data.points_lk_rows; ++i) {
        delete[] points_data.points_lk[i];
    }
    delete[] points_data.points_lk;
    for (int i = 0; i < points_data.points_orb_rows; ++i) {
        delete[] points_data.points_orb[i];
    }
    delete[] points_data.points_orb;
    points_data.points_lk_rows = points.points_lk_rows;
    points_data.points_orb_rows = points.points_orb_rows;

    points_data.points_lk = new carina_lk_point *[points_data.points_lk_rows];
    for (int i = 0; i < points_data.points_lk_rows; ++i) {
        points_data.points_lk_cols[i] = points.points_lk_cols[i];
        points_data.points_lk[i] = new carina_lk_point[points_data.points_lk_cols[i]];
        for (int j = 0; j < points_data.points_lk_cols[i]; ++j) {
            points_data.points_lk[i][j].id = points.points_lk[i][j].id;
            points_data.points_lk[i][j].x = points.points_lk[i][j].x;
            points_data.points_lk[i][j].y = points.points_lk[i][j].y;
        }
    }

    points_data.points_orb = new carina_orb_point *[points_data.points_orb_rows];
    for (int i = 0; i < points_data.points_orb_rows; ++i) {
        points_data.points_orb_cols[i] = points.points_orb_cols[i];
        points_data.points_orb[i] = new carina_orb_point[points_data.points_orb_cols[i]];
        for (int j = 0; j < points_data.points_orb_cols[i]; ++j) {
            points_data.points_orb[i][j].id = points.points_orb[i][j].id;
            points_data.points_orb[i][j].x = points.points_orb[i][j].x;
            points_data.points_orb[i][j].y = points.points_orb[i][j].y;
            points_data.points_orb[i][j].angle = points.points_orb[i][j].angle;
            points_data.points_orb[i][j].octave = points.points_orb[i][j].octave;
            points_data.points_orb[i][j].response = points.points_orb[i][j].response;
            for (int k = 0; k < 32; ++k) {
                points_data.points_orb[i][j].desc[k] = points.points_orb[i][j].desc[k];
            }
        }
    }
    points_ts = ts;
}

void CarinaA1088EventCallBack(const uint8_t uc_event) {
    std::lock_guard<std::mutex> auto_lock(event_mtx);
    event = uc_event;
}

int main(int argc, char **argv) {
    signal(SIGINT, signal_handle);

    char c;
    std::string custom_config_path = "./custom_config.yaml";
    std::string database_path = "./database.bin";

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("vio");

    node->declare_parameter("custom_config_path", custom_config_path);
    node->declare_parameter("database_path", database_path);
    
    custom_config_path = node->get_parameter("custom_config_path").as_string();
    database_path = node->get_parameter("database_path").as_string();

    std::cout << custom_config_path << std::endl;
    std::cout << database_path << std::endl;

    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    //auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>("path", 10);
    auto left_image_pub = node->create_publisher<sensor_msgs::msg::Image>("left_gray_image", 10);
    auto right_image_pub = node->create_publisher<sensor_msgs::msg::Image>("right_gray_image", 10);
    
    rclcpp::Rate loop_rate(100);
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = 
        std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);
    std::ostringstream timestamp;
    timestamp << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
    
    std::string file_path = timestamp.str() + "_VIO_6DOF_data.txt";

    std::ofstream outFile;
    outFile.open(file_path);
    
    if (!outFile.is_open()) {
        std::cerr << "Unable to open file!" << std::endl;
        return 1;
    }

    std::string custom_config;
    std::ifstream configFile(custom_config_path, std::ios::in);
    if (configFile.is_open()) {
        std::stringstream buffer;
        buffer << configFile.rdbuf();
        configFile.close();
        custom_config = buffer.str();
    }

    carina_a1088_init(const_cast<char *>(custom_config.c_str()), const_cast<char*>(database_path.c_str()));
    carina_a1088_start(
        CarinaA1088PoseCallBack,
        CarinaA1088VsyncCallBack,
        CarinaA1088ImuCallBack,
        CarinaA1088CameraCallBack,
        CarinaA1088PointsCallBack,
        CarinaA1088EventCallBack
    );
    carina_a1088_resume();

    while (rclcpp::ok() && !shutdown_requested) {
        sensor_msgs::msg::Imu imu_msg;
        geometry_msgs::msg::PoseStamped pose_stamped;
        sensor_msgs::msg::Image::SharedPtr left_msg, right_msg;

        // 发布 /odom 和 tf
        {
            std::lock_guard<std::mutex> auto_lock(pose_mtx);
            if (pose_ts > 0) {
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = node->now();
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_link";

                odom_msg.pose.pose.position.x = pose_data[12];
                odom_msg.pose.pose.position.y = pose_data[13];
                odom_msg.pose.pose.position.z = pose_data[14];

                double R[9] = {
                    pose_data[0], pose_data[4], pose_data[8],
                    pose_data[1], pose_data[5], pose_data[9],
                    pose_data[2], pose_data[6], pose_data[10]
                };
                auto quat = rotationMatrixToQuaternion(R);

                odom_msg.pose.pose.orientation.x = quat.x();
                odom_msg.pose.pose.orientation.y = quat.y();
                odom_msg.pose.pose.orientation.z = quat.z();
                odom_msg.pose.pose.orientation.w = quat.w();

                // 添加协方差
                for (int i = 0; i < 36; ++i) {
                    odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
                }

                odom_pub->publish(odom_msg);

                geometry_msgs::msg::TransformStamped odom_to_base;
                odom_to_base.header.stamp = node->now();
                odom_to_base.header.frame_id = "odom";
                odom_to_base.child_frame_id = "base_link";

                odom_to_base.transform.translation.x = pose_data[12];
                odom_to_base.transform.translation.y = pose_data[13];
                odom_to_base.transform.translation.z = pose_data[14];

                odom_to_base.transform.rotation.x = quat.x();
                odom_to_base.transform.rotation.y = quat.y();
                odom_to_base.transform.rotation.z = quat.z();
                odom_to_base.transform.rotation.w = quat.w();

                tf_broadcaster_->sendTransform(odom_to_base);

                pose_ts = -1;
            }
        }

        // 发布 IMU 数据
        // {
        //     std::lock_guard<std::mutex> auto_lock(imu_mtx);
        //     if (imu_ts > 0 && imu_data.size() == 6) {
        //         imu_msg.header.stamp = node->now();
        //         imu_msg.header.frame_id = "imu_link";

        //         imu_msg.angular_velocity.x = imu_data[0];
        //         imu_msg.angular_velocity.y = imu_data[1];
        //         imu_msg.angular_velocity.z = imu_data[2];

        //         imu_msg.linear_acceleration.x = imu_data[3];
        //         imu_msg.linear_acceleration.y = imu_data[4];
        //         imu_msg.linear_acceleration.z = imu_data[5];

        //         imu_msg.orientation.x = 0.0;
        //         imu_msg.orientation.y = 0.0;
        //         imu_msg.orientation.z = 0.0;
        //         imu_msg.orientation.w = 1.0;

        //         imu_pub->publish(imu_msg);

        //         imu_ts = -1;
        //     }
        // }

        // 发布相机图像
        {
            std::lock_guard<std::mutex> auto_lock(image_mtx);
            if (image_ts > 0) {
                std::cout << "image_ts ts: " << image_ts << std::endl;
                auto header = std_msgs::msg::Header();
                header.stamp = node->now();
                header.frame_id = "map";
                left_msg = cv_bridge::CvImage(header, "mono8", left_image).toImageMsg();
                right_msg = cv_bridge::CvImage(header, "mono8", right_image).toImageMsg();
                left_image_pub->publish(*left_msg);
                right_image_pub->publish(*right_msg);
                image_ts = -1;
            }
        }

        pose_pub->publish(pose_stamped);
        path_pub->publish(path);
        rclcpp::spin_some(node);
        loop_rate.sleep();

        if (kbhit())
        {
            c = fgetc(stdin);
            std::cout << std::endl;
            if (c == 'q')
                break;
        }
    }

    outFile.close();

    carina_a1088_pause();
    carina_a1088_stop();
    carina_a1088_release();
    rclcpp::shutdown();

    return 0;
}