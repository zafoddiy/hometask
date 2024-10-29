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
    rvizconfig = LaunchConfiguration(
        "rvizconfig",
        default=os.path.join(
            get_package_share_directory(package_name),
            "config",
            "task5.2_config.rviz",
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

    #image_pub_node = Node(
    #    package="ias0220_231899",
    #    executable="image_publisher",
    #    name="image_publisher",
    #    output="screen"
    #)

    #camera_calib_node = Node(
    #    package="ias0220_231899",
    #    executable="camera_calibration",
    #    name="camera_calibration",
    #    output="screen"
    #)

    #static_transform_publisher_node = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    arguments = ["--frame-id", "map", "--child-frame-id", "odom"],
    #)

    gazebo_spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="differential_robot_simu_task4_part2",
        namespace="my_robot",
        output="screen",
        arguments=["-topic", "/robot_description", "-entity", "observerbot", "-x", "0.6", "-y", "-7.5", "-Y", "1.6"],
    )

    object_recognition_node = Node(
        package="ias0220_231899",
        executable="object_recognition",
        name="object_recognition",
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        rviz_node,
        robot_state_publisher_node,
        gazebo_spawn_entity_node,
        object_recognition_node,
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([get_package_share_directory('setup_gazebo_ias0220'), '/launch/gazebo.launch.py']),
        #    launch_arguments={
        #        'xacro_file': xacro_file,
        #    }.items()
        #),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('machine_vision_part2'), '/launch/mvt_main.launch.py']),
        )             
    ])