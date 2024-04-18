import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = 'srv_services'

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

    mount_node = Node(
        package=package_name,
        executable='mount_position',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},]
    )
    speed_node = Node(
        package=package_name,
        executable='set_speed',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},]
    )
 

    return launch.LaunchDescription([
       robot_name_launch_arg,
       robot_ip_launch_arg,
       mount_node,
       speed_node,
       #LogInfo(msg=LaunchConfiguration('robot_ip')),
       #LogInfo(msg=LaunchConfiguration('robot_name')),
    ])