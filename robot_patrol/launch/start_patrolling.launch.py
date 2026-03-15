import launch
import launch_ros.actions

def generate_launch_description():
    patrol_start_node = launch_ros.actions.Node(
        package="robot_patrol",
        executable="patrol_executable",
        arguments=[],
        output="screen",)
    return launch.LaunchDescription([
        patrol_start_node])