import launch
from launch_ros.actions import Node

import ament_index_python.packages as pk
import ament_index_python.resources as pr
import os

package_name = 'lidar-camera-calibration'

def generate_launch_description():
    package_dir = pk.get_package_share_directory(package_name)
    return launch.LaunchDescription([
        Node(
            package=package_name,
            executable='lidar_camera_projection',
            name='lidar_camera_projection',
            output='screen',
            parameters=[
                {"calibration_params_path" : os.path.join(package_dir, "params", "calibration_params.json")},
                {"camera_params_path" : os.path.join(package_dir, "params", "camera_intrinsic.json")},
            ]),
  ])