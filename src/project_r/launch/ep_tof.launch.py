from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PathJoinSubstitution([
        FindPackageShare('robomaster_ros'),
        'launch',
        'main.launch'
      ]),
      launch_arguments={
        'model': 'ep',
        'name': 'rm0',
        'tof_0': 'true',
        'tof_0_parent': 'base_link',
        'tof_0_xyz': '-0.10703 -0.0996 0.13077',
        'tof_0_rpy': '0 0 -2.79253',
        'tof_1': 'true',
        'tof_1_parent': 'base_link',
        'tof_1_xyz': '0.10297 -0.0996 0.13077',
        'tof_1_rpy': '0 0 -0.34907',
        'tof_2': 'true',
        'tof_2_parent': 'base_link',
        'tof_2_xyz': '-0.10703 0.0996 0.13077',
        'tof_2_rpy': '0 0 2.79253',
        'tof_3': 'true',
        'tof_3_parent': 'base_link',
        'tof_3_xyz': '0.10297 0.0996 0.13077',
        'tof_3_rpy': '0 0 0.34907'
      }.items()
    )
  ])
