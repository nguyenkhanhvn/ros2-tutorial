from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_path("bumperbot_controller"), "config", "joystick.yaml")]
    )
    
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_path("bumperbot_controller"), "config", "joy_teleop.yaml")]
    )
    
    
    return LaunchDescription([
        joy_node,
        joy_teleop
    ])
    
    
    
    
    