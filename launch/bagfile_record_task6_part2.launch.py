import launch
from ament_index_python.packages import get_packages_with_prefixes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

package_name = "ias0220_231899"


def generate_launch_description():

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
    bag_path = PathJoinSubstitution([where_bag, 'rosbag_to_light'])

    bag_record = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '/tf', '/tf_static', '/imu', '/distance', '/diff_cont/cmd_vel_unstamped', '-o',
            bag_path
        ],
        output='screen'
    )

    return LaunchDescription([
        static_transform_publisher_node,
        serial_interface_node,
        bag_record
    ])
