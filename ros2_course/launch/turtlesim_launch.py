import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    turtlesim_node = launch_ros.actions.Node(
        package='turtlesim', executable='turtlesim_node', output='screen')

    turtlesim_controller_node = launch_ros.actions.Node(
        package='ros2_course', executable='turtlesim_controller', output='screen')

    return launch.LaunchDescription([
        turtlesim_node,
        turtlesim_controller_node
    ])

