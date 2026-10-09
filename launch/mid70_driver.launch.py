from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mid70_driver_node = Node(
        package="mid360_driver",
        executable="mid70_driver_node",
        name="mid70_driver",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("mid360_driver"),
                    "config",
                    "mid70_param.yaml",
                ]
            )
        ],
    )

    return LaunchDescription([mid70_driver_node])
