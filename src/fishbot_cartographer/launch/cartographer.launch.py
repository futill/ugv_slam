import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare(package='fishbot_cartographer').find('fishbot_cartographer')
    vio_odom = get_package_share_directory('oakchina_vio_package')
    driver_dir = get_package_share_directory('lslidar_driver')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'fishot_2d.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    configuration_directory = LaunchConfiguration(
        'configuration_directory', default=os.path.join(pkg_share, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_2d.lua')

    # 地图文件路径（你需要确认这个路径下的 .yaml 文件是否存在）
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径

    lslidar_driver_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([driver_dir,'/launch','/lsn10_launch.py']),
        )
    rviz_vio_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vio_odom,'/launch','/oakchina_vio.launch.py']),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': True}],
        output='screen'
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
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])



    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'fishbot_cartographer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(lslidar_driver_node)
    ld.add_action(rviz_vio_odom)
    ld.add_action(cartographer_node)
    # ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
