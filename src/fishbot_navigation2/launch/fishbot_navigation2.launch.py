'''
作者: 小鱼
公众号: 鱼香ROS
QQ交流群: 2642868461
描述: Nav2 launch启动文件
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    vio_odom = get_package_share_directory('oakchina_vio_package')
    driver_dir = get_package_share_directory('lslidar_driver')
    urdf_dir = os.path.join(fishbot_navigation2_dir, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'fishot_2d.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(fishbot_navigation2_dir,'maps','map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(fishbot_navigation2_dir,'param','fishbot_nav2.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')
    configuration_directory = LaunchConfiguration(
        'configuration_directory', default=os.path.join(fishbot_navigation2_dir, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_2d.lua')
    # 地图文件路径（你需要确认这个路径下的 .yaml 文件是否存在）
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    
    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}],
        output = 'screen'
        )
    time_sync_node = Node(
        package = 'fishbot_cartographer',
        executable = 'time_sync_node',
        output = 'screen'
        )
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ]
    )
    uart_controller = Node(
        package='fishbot_navigation2',
        executable='uart_controller',
        name='uart_controller',
        output='screen',
    )
    lslidar_driver_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([driver_dir,'/launch','/lsn10_launch.py']),
        )
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([fishbot_navigation2_dir,'/launch','/nav2_cartographer.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_path,
                'params_file': nav2_param_path}.items(),
        )
    rviz_vio_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vio_odom,'/launch','/oakchina_vio.launch.py']),
    )
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    return LaunchDescription([
        robot_state_publisher_node,
        time_sync_node,
        lslidar_driver_node,
        rviz_vio_odom,
        cartographer_node,
        # occupancy_grid_node,
        nav2_bringup_launch,
        uart_controller,
        rviz_node])
