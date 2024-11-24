import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

package_name = "ias0220_231899"

def generate_launch_description():
    package_path = os.path.join(get_package_share_directory(package_name))

    # Parse the urdf with xacro
    xacro_file = os.path.join(package_path, "urdf", "differential_robot_simu_task4_part2.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml()}

    # Define launch arguments
    rvizconfig = LaunchConfiguration(
        "rvizconfig",
        default=os.path.join(
            get_package_share_directory(package_name),
            "config",
            "task4_config.rviz",
        ),
    )

    # Define nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["--display-config", rvizconfig],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[params],
    )

    teleop_twist_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/diff_cont/cmd_vel_unstamped'),
        ],
    )

    encoder_node = Node(
        package="encoders_pkg",
        executable="encoders_node",
        name="encoders_pkg",
    )

    odometry_node = Node(
        package="ias0220_231899",
        executable="odometry",
        name="odometry",
        output="screen"
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        teleop_twist_node,
        encoder_node,
        odometry_node,
        static_transform_publisher_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('setup_gazebo_ias0220'), '/launch/gazebo.launch.py']),
            launch_arguments={
                'xacro_file': xacro_file,
            }.items()
        ),             
    ])