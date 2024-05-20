import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path= get_package_share_path('my_robot_bringup')
    my_robot_description_path = get_package_share_path('my_robot_description')
    
    urdf_path = os.path.join(my_robot_description_path, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(my_robot_description_path, 'rviz', 'simple_robot_car.rviz')
    gazebo_world_path = os.path.join(pkg_path, 'worlds', 'test.world')
    
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_path("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={'world': gazebo_world_path}.items()
    )
    
    spawn_robot_node = Node (
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot"],
        # output="screen"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    
    return LaunchDescription([
        robot_state_publisher_node,
        start_gazebo,
        spawn_robot_node,
        # rviz_node
    ])
    
    
    
    
    