#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Define the planner node
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Define the controller node
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Define the smoother node
    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add nodes to the launch description
    ld.add_action(planner_node)
    ld.add_action(controller_node)
    ld.add_action(smoother_node)

    # Define lifecycle transition commands
    configure_planner = TimerAction(
        period=1.0,  # Time before configuring
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'configure'],
            output='screen'
        )]
    )

    configure_controller = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'configure'],
            output='screen'
        )]
    )

    configure_smoother = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'configure'],
            output='screen'
        )]
    )

    activate_planner = TimerAction(
        period=2.0,  # Adjust if necessary
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/planner_server', 'activate'],
            output='screen'
        )]
    )

    activate_controller = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'],
            output='screen'
        )]
    )

    activate_smoother = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/smoother_server', 'activate'],
            output='screen'
        )]
    )

    # Add lifecycle activation actions to the launch description
    ld.add_action(configure_planner)
    ld.add_action(configure_controller)
    ld.add_action(configure_smoother)

    # Ensure the activation commands run after configuration
    ld.add_action(TimerAction(period=2.0, actions=[activate_planner]))
    ld.add_action(TimerAction(period=2.0, actions=[activate_controller]))
    ld.add_action(TimerAction(period=2.0, actions=[activate_smoother]))

    return ld
