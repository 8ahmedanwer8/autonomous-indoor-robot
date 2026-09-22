import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    simulation_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..')
    )

    default_params_file = os.path.join(
        simulation_dir,
        'config',
        'elegoo_nav2_params.yaml'
    )

    nav_to_pose_bt = os.path.join(
        simulation_dir,
        'config',
        'elegoo_nav_to_pose.xml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to Nav2 params file'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the Nav2 stack'
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),

        GroupAction([
            SetParameter('use_sim_time', use_sim_time),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),

            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[
                    params_file,
                    {'default_nav_to_pose_bt_xml': nav_to_pose_bt},
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    params_file,
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ]),
    ])
