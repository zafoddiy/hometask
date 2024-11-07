import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# import xacro

package_name = "ias0220_231899"


def generate_launch_description():

    # Define launch arguments
    rvizconfig = LaunchConfiguration(
        "rvizconfig",
        default=os.path.join(
            get_package_share_directory(package_name),
            "config",
            "task6.1_config.rviz",
        ),
    )

    # Define nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["--display-config", rvizconfig],
    )

    return LaunchDescription([
        rviz_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(
                'ias0220_sensors'), '/launch/sensors_rviz.launch.py']),
            launch_arguments={
                'which_bag': 'recorded',
            }.items()
        )
    ])
