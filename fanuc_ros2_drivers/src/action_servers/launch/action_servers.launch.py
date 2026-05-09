import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

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
        package='action_servers',
        executable='cart_pose_server',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    convey_node = Node(
        package='action_servers',
        executable='convey_server',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    joint_node = Node(
        package='action_servers',
        executable='joint_pose_server',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    onrobot_node = Node(
        package='action_servers',
        executable='onrobot_server',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    schunk_node = Node(
        package='action_servers',
        executable='schunk_server',
        #namespace=robot_name,
        parameters=[{"robot_ip": robot_ip,
                     "robot_name": robot_name,},],
        respawn=True,
        respawn_delay=4,
    )
    sjoint_node = Node(
        package='action_servers',
        executable='single_joint_server',
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
       convey_node,
       joint_node,
       onrobot_node,
       schunk_node,
       sjoint_node,
       LogInfo(msg=LaunchConfiguration('robot_ip')),
       LogInfo(msg=LaunchConfiguration('robot_name')),
    ])