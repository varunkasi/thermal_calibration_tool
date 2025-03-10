#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    data_dir_arg = DeclareLaunchArgument(
        'data_dir', 
        default_value=os.path.join(EnvironmentVariable('HOME'), 'thermal_calibration_data'),
        description='Directory for storing calibration data'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/mono16_converter/image',
        description='Thermal image topic to subscribe to'
    )
    
    launch_rqt_arg = DeclareLaunchArgument(
        'launch_rqt',
        default_value='true',
        description='Whether to launch the RQT interface'
    )

    # Get the path to the thermal calibration perspective file
    thermal_calibration_pkg = FindPackageShare('thermal_calibration_rqt')
    perspective_path = PathJoinSubstitution(
        [thermal_calibration_pkg, 'resource', 'thermal_calibration.perspective']
    )
    
    # Define thermal calibration node
    thermal_node = Node(
        package='thermal_calibration_rqt',
        executable='thermal_calibration_node',
        name='thermal_calibration_node',
        output='screen',
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'thermal_image_topic': LaunchConfiguration('image_topic')
        }]
    )
    
    # Define RQT with thermal calibration plugin - force load our plugin explicitly
    rqt_process = ExecuteProcess(
        cmd=['rqt', 
             '--force-discover', 
             '--perspective-file', perspective_path,
             '--clear-config',
             '-s', 'thermal_calibration_rqt.thermal_calibration_plugin.ThermalCalibrationPlugin'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rqt'))
    )
    
    return LaunchDescription([
        data_dir_arg,
        image_topic_arg,
        launch_rqt_arg,
        thermal_node,
        rqt_process
    ])