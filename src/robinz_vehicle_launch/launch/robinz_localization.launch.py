from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # config and args
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = ['amcl']

    remappings = [('/tf', 'tf'),('/tf_static', 'tf_static')]
    args_namespace = DeclareLaunchArgument(
            'namespace', default_value='',
            description='fonfig namespace'
    )
    args_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')
    args_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        # parameters=[configured_params],
        # remappings=remappings
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        namespace=namespace,
        arguments=['-d', os.path.join(get_package_share_directory('robinz_vehicle_launch'), 'rviz', 'simulation.rviz')]
    )
    
    # Finalize launch description
    ld = LaunchDescription()
    ld.add_action(args_namespace)
    ld.add_action(args_use_sim_time)
    ld.add_action(args_autostart)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(amcl_node)
    ld.add_action(rviz_node)

    return ld