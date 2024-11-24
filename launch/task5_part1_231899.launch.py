import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
            "task5_config.rviz",
        ),
    )

    # Define nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["--display-config", rvizconfig],
    )

    image_pub_node = Node(
        package="ias0220_231899",
        executable="image_publisher",
        name="image_publisher",
        output="screen"
    )

    camera_calib_node = Node(
        package="ias0220_231899",
        executable="camera_calibration",
        name="camera_calibration",
        output="screen"
    )

    image_proc_node = ComposableNodeContainer(
        name="image_proc",
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin='image_proc::RectifyNode',
                remappings=[
                    ('image','image_raw')
                ],
            ),
        ],
        output='both'
    )

    return LaunchDescription([
        rviz_node,
        image_pub_node,
        camera_calib_node,
        image_proc_node,
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([get_package_share_directory('setup_gazebo_ias0220'), '/launch/gazebo.launch.py']),
        #    launch_arguments={
        #        'xacro_file': xacro_file,
        #    }.items()
        #),             
    ])