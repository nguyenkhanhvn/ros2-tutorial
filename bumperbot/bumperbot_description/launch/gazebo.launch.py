import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bumperbot_description_path = get_package_share_path('bumperbot_description')
    bumperbot_description_prefix = get_package_prefix('bumperbot_description')
    
    model_path = os.path.join(bumperbot_description_path, "models")
    model_path += os.pathsep + os.path.join(bumperbot_description_prefix, "share")
    
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
        bumperbot_description_path, "urdf", "bumperbot.urdf.xacro"
    ), description="Absolute path to robot urdf file")
    
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]))
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path("gazebo_ros"), "launch", "gzserver.launch.py")
    ))
    
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path("gazebo_ros"), "launch", "gzclient.launch.py")
    ))
    
    start_gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path("gazebo_ros"), "launch", "gazebo.launch.py")
    ))
    
    spawn_robot_node = Node (
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bumperbot", "-topic", "robot_description"],
        output="screen"
    )
    
    
    
    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher_node,
        # start_gazebo_server,
        # start_gazebo_client,
        start_gazebo,
        spawn_robot_node
    ])
    
    
    
    
    