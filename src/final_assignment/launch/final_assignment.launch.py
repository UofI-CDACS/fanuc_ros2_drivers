import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='noName',
        description='Name of the robot these nodes are attached to',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.29.208.1',
        description='IP address of the robot',
    )

    camera_node = Node(
        package='dice_pipeline',
        executable='camera_node',
        name='camera_node',
        respawn=True,
        respawn_delay=15,
    )

    return launch.LaunchDescription([
        robot_name_arg,
        robot_ip_arg,
        camera_node,
    ])
