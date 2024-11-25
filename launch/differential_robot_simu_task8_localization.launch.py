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

    map_file = os.path.join(
        get_package_share_directory(package_name),
        "map",
        "room.yaml"
    )

    nav2_yaml = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "nav2_params.yaml"
    )

    # Define nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["--display-config", rvizconfig],
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )

    controller_node = Node(
        package="ias0220_231899",
        executable="simple_control",
        parameters=[yaml_config],
        output="screen"
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server', 
        name='map_server',
        output='screen', 
        parameters=[{'use_sim_time': True}, 
             {'yaml_filename':map_file}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper', 
        output='screen',
        parameters=[{'use_sim_time': True}, 
             {'autostart': True}, 
             {'node_names': ['map_server']}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl', 
        name='amcl', 
        output='screen',
        parameters=[nav2_yaml]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        rviz_node,
        controller_node,
        static_transform_publisher_node,
        map_server_node,
        lifecycle_manager_node,
        amcl_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory(
                'setup_gazebo_ias0220'), '/launch/gazebo.launch.py']),
            launch_arguments={
                'xacro_file': xacro_file,
            }.items()
        ),
    ])