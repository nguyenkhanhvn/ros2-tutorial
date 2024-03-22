from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    cpp_topic_publisher_node = Node(
        package="tutorial_cpp_pkg",
        executable="topic_publisher",
        name="cpp_topic_publisher"
    )
    cpp_topic_subscription_node = Node(
        package="tutorial_cpp_pkg",
        executable="topic_subscription",
        name="cpp_topic_subscription",
        remappings=[
            ("cpp_tutorial_topic", "tutorial_topic")
        ]
    )
    
    py_topic_publisher_node = Node(
        package="tutorial_py_pkg",
        executable="topic_publisher",
        name="py_topic_publisher",
        remappings=[
            ("py_tutorial_topic", "tutorial_topic")
        ],
        parameters=[
            {"param_int64": 999},
            {"param_string": "THIS IS PYTHON"}
        ]
    )
    py_topic_subscription_node = Node(
        package="tutorial_py_pkg",
        executable="topic_subscription",
        name="py_topic_subscription"
    )
    
    ld.add_action(py_topic_publisher_node)
    ld.add_action(cpp_topic_subscription_node)
    return ld