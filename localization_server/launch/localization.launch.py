import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare the map_file argument
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_real.yaml',
        description='Name of the map file to load')

    # Get the map file from the launch argument
    map_file_name = LaunchConfiguration('map_file')
    amcl_config = ''
    use_sim_time = False
    

    # Paths for the AMCL config files
    amcl_config_real = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')
    amcl_config_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')

    if map_file_name == 'warehouse_map_real.yaml':
        amcl_config = amcl_config_real
        odom_frame = 'robot_odom'
        use_sim_time = False
    else:
        amcl_config = amcl_config_sim
        odom_frame = 'odom'
        use_sim_time = True

    # Path for the RViz config file
    rviz_config_file = os.path.join(get_package_share_directory('localization_server'), 'config', 'localizer_config.rviz')

    # Path for the map file
    map_file_path = [os.path.join(get_package_share_directory('map_server'), 'config'), '/', map_file_name]

    # Dynamically assign AMCL config and odom frame based on map file
    amcl_config = LaunchConfiguration('amcl_config', default=amcl_config_sim)
    odom_frame = LaunchConfiguration('odom_frame', default='odom')



    # Print statements for debugging, using LogInfo to print runtime substitution values
    log_info_map = LogInfo(msg=['Using map file: ', map_file_name])
    log_info_amcl = LogInfo(msg=['Using AMCL config file: ', amcl_config])
    log_info_odom = LogInfo(msg=['Using odom frame: ', odom_frame])
    log_info_time = LogInfo(msg=['Sim Time: ', str(use_sim_time)])


    return LaunchDescription([
        # Declare the map_file argument
        declare_map_file_cmd,

        # Log info statements for debugging
        log_info_map,
        log_info_amcl,
        log_info_odom,
        log_info_time,

        # Start the map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_file_path}]
        ),

        # Start the AMCL node for localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config]
        ),

        # Start RViz2 to visualize the map
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_file]
        ),

        # Start the lifecycle manager for managing the nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        
        # Add a static transform publisher to link 'map' and 'odom' or 'robot_odom'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
            parameters=[{'use_sim_time': True}]
        ),
    ])




