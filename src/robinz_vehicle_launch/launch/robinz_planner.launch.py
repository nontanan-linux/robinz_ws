from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path for parameter files
    bringup_dir = get_package_share_directory('robinz_vehicle_launch')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_planner.yaml')

    # Set Environment Variables
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    ))

    # Rewritten YAML file with param substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Launch the lifecycle manager
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[configured_params],
        remappings=[('/lifecycle_manager/activate', '/activate')]
    ))

    # Launch Nav2 Planner, Controller, Recovery, and Costmap nodes
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('/plan', '/path_plan')]
    ))

    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params]
    ))

    # ld.add_action(Node(
    #     package='nav2_recovery',
    #     executable='recovery_server',
    #     name='recovery_server',
    #     output='screen',
    #     parameters=[configured_params]
    # ))

    ld.add_action(Node(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[configured_params]
    ))

    ld.add_action(Node(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[configured_params]
    ))

    return ld
