# 项目名称

NAV2_STATIC_MAP_GO

## 📦 功能特性

- 使用Cartographer融合imu，odom里程计，雷达进行SLAM纯定位
- 使用navigation2获取静态map，在虚拟地图中进行导航避障
- 当前实现情况（已实现）
   tf:map->odom->base_link->laser
   map->odom由Cartographer发布，odom->base_link由oak-vio-kit视觉里程计发布。
   已具备导航规划能力

## 🛠️ 安装与使用

### 安装

# 克隆仓库
git clone https://github.com/futill/ugv_slam.git
cd ugv_slam /colcon build

# 安装依赖（以 Node.js 项目为例）
sudo apt install ros-humble-navigation*
sudo apt install ros-humble-cartographer*

ros2 launch fishbot_navigation2 fishbot_navigation2

## 贡献指南
OAKChina-vio:
官方Gitee地址：https://gitee.com/oakchina/oakchina-vio