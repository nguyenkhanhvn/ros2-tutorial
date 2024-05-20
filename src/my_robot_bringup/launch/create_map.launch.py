import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    
    pkg_path = get_package_share_path('my_robot_bringup')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'simple_robot_car.rviz')


    # nav2
    start_nav2_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_path("nav2_bringup"), 'launch', 'navigation_launch.py')]))

    # slam_toolbox
    start_slam_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_path("slam_toolbox"), 'launch', 'online_async_launch.py')]))
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    ld = LaunchDescription()
    
    # Declare the launch options
    # ld.add_action(start_nav2_cmd)
    ld.add_action(start_slam_cmd)
    ld.add_action(rviz_node)
    
    
    return ld