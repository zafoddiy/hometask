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
            "default_task9.rviz",
        ),
    )

    # Define nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["--display-config", rvizconfig],
        remappings=[
            ('/tf', '/tb4_34/tf'),
            ('/tf_static', '/tb4_34/tf_static'),
        ],
    )

    square_movement_node = Node(
        package="ias0220_231899",
        executable="square_movement",
        name="square_node",
    )

    return LaunchDescription([
        rviz_node,
        square_movement_node,
    ])
