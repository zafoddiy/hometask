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
    xacro_file = os.path.join(package_path, "urdf", "differential_robot_simu_task5_part2.urdf")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml(), "use_sim_time": use_sim_time}

    # Define launch arguments

    # Define nodes

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[params],
    )

    steering_node = Node(
        package="ias0220_231899",
        executable="steering_node",
        name="steering_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        steering_node,
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([get_package_share_directory('setup_gazebo_ias0220'), '/launch/gazebo.launch.py']),
           launch_arguments={
               'xacro_file': xacro_file,
           }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(
                'ias0220_sensors'), '/launch/sensors_rviz.launch.py']),
            launch_arguments={
                'which_bag': 'bag2',
            }.items()
        ),         
    ])