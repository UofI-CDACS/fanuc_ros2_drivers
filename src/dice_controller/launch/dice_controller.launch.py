import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='noName',
        description='Name of the robot (used as ROS topic namespace prefix)',
    )
    robot_name = LaunchConfiguration('robot_name')

    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='OpenCV VideoCapture index of the overhead camera (0 = default)',
    )
    camera_index = LaunchConfiguration('camera_index')

    camera_node = Node(
        package='dice_controller',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'robot_name':   robot_name,
            'camera_index': camera_index,
            'save_dir':     '/tmp/dice_images',
            'publish_hz':   10.0,
        }],
        respawn=True,
        respawn_delay=4,
    )

    # The master node is launched separately (ros2 run) after the servers are
    # ready, because it runs a one-shot sequential program.  Uncomment below if
    # you want it started automatically alongside the camera.
    #
    # master_node = Node(
    #     package='dice_controller',
    #     executable='master_node',
    #     name='dice_master',
    #     parameters=[{
    #         'robot_name': robot_name,
    #         # Set Cartesian positions here or pass as -p arguments at runtime
    #     }],
    # )

    return launch.LaunchDescription([
        robot_name_arg,
        camera_index_arg,
        camera_node,
        # master_node,  # uncomment when positions are calibrated
    ])
