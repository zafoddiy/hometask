import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    steering_node = Node(
        package="ias0220_231899",
        executable="steering_node",
        name="steering_node",
        output="screen",
    )

    serial_interface_node = Node(
        package="ias0220_sensors",
        executable="serial_interface",
        name="serial_interface",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        rviz_node,
        steering_node,
        serial_interface_node,
    ])
