import launch
from launch_ros.actions import Node

import ament_index_python.packages as pk
import os

package_name = 'lidar-camera-calibration'

def generate_launch_description():
    package_dir = pk.get_package_share_directory(package_name)
    return launch.LaunchDescription([
        Node(
            package=package_name,
            executable='calibration.py',
            name='lidar_camera_calibration',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"camera_params_path" : os.path.join(package_dir, "params", "camera_intrinsic.json")},
            ]),

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'calibration.rviz')]),
  ])