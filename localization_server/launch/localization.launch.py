import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the 'map_file' argument
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_real.yaml',
        description='Name of the map file to load (determines which launch file to use)'
    )

    # Define launch configurations
    map_file = LaunchConfiguration('map_file')

    # Path to the launch files
    localization_sim_launch = os.path.join(
        get_package_share_directory('localization_server'),
        'launch',
        'localization_sim.launch.py'
    )
    
    localization_real_launch = os.path.join(
        get_package_share_directory('localization_server'),
        'launch',
        'localization_real.launch.py'
    )

    # Conditional launch of the appropriate file based on the 'map_file' argument
    return LaunchDescription([
        declare_map_file_cmd,

        # IfCondition to check if the map_file is warehouse_map_real.yaml, launch the real localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_real_launch),
            condition=IfCondition(map_file.equals('warehouse_map_real.yaml'))
        ),

        # Otherwise, launch the simulation localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_sim_launch),
            condition=IfCondition(map_file.equals('warehouse_map_sim.yaml'))
        ),
    ])
