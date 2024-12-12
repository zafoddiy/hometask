import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = "ias0220_231899"


def generate_launch_description():

    # Define launch arguments
    rvizconfig = LaunchConfiguration(
        "rvizconfig",
        default=os.path.join(
            get_package_share_directory(package_name),
            "config",
            "task8_config.rviz",
        ),
    )

    yaml_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "simple_control_v2.yaml"
    )

    # Define nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["--display-config", rvizconfig],
    )

    controller_node = Node(
        package="ias0220_231899",
        executable="simple_control",
        parameters=[yaml_config],
        output="screen"
    )

    return LaunchDescription([
        rviz_node,
        controller_node,
    ])
