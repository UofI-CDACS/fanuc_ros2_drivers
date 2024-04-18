import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = 'msg_publishers'

    robot_name_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='noName',
        description="Name of the robot these nodes will be attached to"
    )
    robot_ip_launch_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value = '172.29.208.1',
        description="IP address of the robot these nodes will be attached to"
    )
    robot_name = LaunchConfiguration('robot_name')
    robot_ip = LaunchConfiguration('robot_ip')

    cart_node = Node(
        package=package_name,
        executable='current_cart',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    grip_node = Node(
        package=package_name,
        executable='current_grip',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    joint_node = Node(
        package=package_name,
        executable='current_joint',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    move_node = Node(
        package=package_name,
        executable='move_check',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    prox_node = Node(
        package=package_name,
        executable='prox_check',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    speed_node = Node(
        package=package_name,
        executable='speed_check',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )

    return launch.LaunchDescription([
       robot_name_launch_arg,
       robot_ip_launch_arg,
       cart_node,
       grip_node,
       joint_node,
       move_node,
       prox_node,
       speed_node,
       #LogInfo(msg=LaunchConfiguration('robot_ip')),
       #LogInfo(msg=LaunchConfiguration('robot_name')),
    ])