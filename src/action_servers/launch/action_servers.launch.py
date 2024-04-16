import launch
import launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    robot_ip = LaunchConfiguration('robot_ip')

    robot_name_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='noName',
        description="Name of the robot these nodes will be attached to"
    )
    robot_ip_launch_arg = DeclareLaunchArgument(
        'robot_ip'
        default_value = '172.29.208.0',
        description="IP address of the robot these nodes will be attached to"
    )

    cart_node = Node(
        package='action_servers',
        executable='cart_pose_server',
        namespace=robot_name,
        parameters=[robot_ip]
        #parameters=[{'robot_ip', LaunchConfiguration('robot_ip')}] #?
    )

    return launch.LaunchDescription([
       
    ])