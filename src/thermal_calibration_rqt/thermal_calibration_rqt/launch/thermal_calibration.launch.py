from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
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
            cmd=['rqt', '--force-discover', '--perspective-file', 
                 'install/thermal_calibration_rqt/share/thermal_calibration_rqt/resource/thermal_calibration.perspective'],
            output='screen'
        )
    ])