import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

package_name = "ias0220_231899"


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    package_path = os.path.join(get_package_share_directory(package_name))

    # Parse the urdf with xacro
    xacro_file = os.path.join(package_path, "urdf",
                              "differential_robot_simu_task5_part2.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml(), "use_sim_time": use_sim_time}

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

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="async_slam_toolbox_node",
        remappings=[
            ('/scan', '/tb4_34/scan'),
            ('/tf', '/tb4_34/tf'),
        ],
    )

    controller_node = Node(
        package="ias0220_231899",
        executable="simple_control",
        parameters=[yaml_config],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        rviz_node,
        controller_node,
        slam_node,
    ])
