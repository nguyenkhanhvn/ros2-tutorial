from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    ld = LaunchDescription()
    
    number_node = LifecycleNode(
        package="tutorial_py_pkg",
        executable="lifecycle_number_publisher",
        name="my_number_publisher",
        namespace=""
    )
    
    manager_node = Node(
        package="tutorial_py_pkg",
        executable="lifecycle_node_manager",
        parameters=[
            {"managed_node_name": "my_number_publisher"}
        ]
    )
    
    ld.add_action(number_node)
    ld.add_action(manager_node)
    
    return ld