import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    pkg_path = get_package_share_path("my_robot_bringup")
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'nav2_demo.rviz')
    # map_path = os.path.join(pkg_path, 'maps', 'turtlebot3_world.yaml')
    map_path = os.path.join(pkg_path, 'maps', 'demomap.yaml')


    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')


    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)
    


    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
        )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=map_path,
        description='Full path to map yaml file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    # Bringup
    # Node(
    #     condition=IfCondition(use_composition),
    #     name='nav2_container',
    #     package='rclcpp_components',
    #     executable='component_container_isolated',
    #     parameters=[nav2_params_file, {'autostart': 'True'}],
    #     arguments=['--ros-args', '--log-level', 'info'],
    #     output='screen',
    # )

    # ------------------------ Start Localization ------------------------#

    start_localization_actions = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['map_server', 'amcl']}]),
        ]
    )

    # --------------------------------------------------------------------#

    # ------------------------ Start Navigation ------------------------#

    start_navigation_actions = GroupAction(
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=False,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['controller_server',
                                            'smoother_server',
                                            'planner_server',
                                            'behavior_server',
                                            'bt_navigator',
                                            'waypoint_follower',
                                            'velocity_smoother']}]),
        ]
    )

    # ------------------------------------------------------------------#

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(start_localization_actions)
    ld.add_action(start_navigation_actions)

    ld.add_action(rviz_node)

    return ld