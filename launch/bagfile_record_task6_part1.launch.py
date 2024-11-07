# import os
import launch
# from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_packages_with_prefixes
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

package_name = "ias0220_231899"


def generate_launch_description():

    # Parse the urdf with xacro
    # xacro_file = os.path.join(package_path, "urdf",
    #                           "differential_robot_simu_task5_part2.urdf")
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {"robot_description": doc.toxml(), "use_sim_time": use_sim_time}

    # Define launch arguments
    # rvizconfig = LaunchConfiguration(
    #     "rvizconfig",
    #     default=os.path.join(
    #         get_package_share_directory(package_name),
    #         "config",
    #         "task6.1_config.rviz",
    #     ),
    # )

    # Define nodes
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz",
    #     arguments=["--display-config", rvizconfig],
    # )

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     parameters=[params],
    # )

    serial_interface_node = Node(
        package="ias0220_sensors",
        executable="serial_interface",
        name="serial_interface",
        output="screen"
    )

    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )

    # use the 'ias0220_sensors package as reference
    pkg_in = {get_packages_with_prefixes()[package_name]}

    # slice the string and remove the last two parts from the list
    pkg_ = pkg_in.pop().split('/')[:-2]

    # create the bag directory path
    where_bag = "/".join(pkg_) + "/bags/"
    print(f'{where_bag}')

    # create the full path to the bag file
    bag_path = PathJoinSubstitution([where_bag, 'recorded'])

    bag_record = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--all', '-o',
            bag_path
        ],
        output='screen'
    )

    return LaunchDescription([
        static_transform_publisher_node,
        serial_interface_node,
        bag_record
    ])
