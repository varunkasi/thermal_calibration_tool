from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the path to the thermal calibration perspective file
    thermal_calibration_pkg = FindPackageShare('thermal_calibration_rqt')
    perspective_path = PathJoinSubstitution(
        [thermal_calibration_pkg, 'resource', 'thermal_calibration.perspective']
    )
    
    return LaunchDescription([
        # Thermal calibration node
        Node(
            package='thermal_calibration_rqt',
            executable='thermal_calibration_node',
            name='thermal_calibration_node',
            output='screen'
        ),
        
        # RQT with thermal calibration plugin
        ExecuteProcess(
            cmd=['rqt', '--force-discover', '--perspective-file', perspective_path],
            output='screen'
        )
    ])