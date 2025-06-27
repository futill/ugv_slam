此包基于lsn10雷达，oak_vio_kit视觉里程计的cartographer_slam
使用方法
1.把除了src外的文件删除
2.执行 sudo apt install ros-humble-cartographer / sudo apt install ros-humble-cartographer-ros / sudo apt install ros-humble-nav2-*/
3.oak_vio_kit 的cmake 要手动修改链接opecv4.2路径，opencv4.2自己手动编译，编译完不需要install
4.重新编译colcon build
