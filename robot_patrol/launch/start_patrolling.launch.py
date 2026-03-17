import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("robot_patrol")
    rviz_config_file = os.path.join(pkg_share, "rviz", "patrol_config.rviz")

    patrol_start_node = launch_ros.actions.Node(
        package="robot_patrol",
        executable="patrol_executable",
        arguments=[],
        output="screen",)
    
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",)

    return launch.LaunchDescription([
        patrol_start_node,
        rviz_node
        ])