# 关于Realsense SDK和Realsense-ros的安装方法
先到https://github.com/IntelRealSense/librealsense/releases/tag/v2.53.1这里下载的zip文件
进行uin zip 解压后
（1）更新 ubuntu
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
（2）安装依赖项
解压好后，进入 librealsense 文件目录，并在该目录下打开终端。
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
（3）安装 librealsense原文件目录下的许可脚本
./scripts/setup_udev_rules.sh
注意：该许可脚本可以通过如下命令移除
  ./scripts/setup_udev_rules.sh --uninstall
源码编译
（1）导航到 librealsense 根目录并运行
mkdir build && cd build
（2）运行CMake
cmake ../ -DCMAKE_BUILD_TYPE=Release
（3）重新编译并安装 librealsense 二进制文件
sudo make uninstall && make clean && make && sudo make install
注意：编译的时候不要插着t265
完成后运行realsense-viewer测试是否连接成功

后续会更新编译内核让t265上电自启
