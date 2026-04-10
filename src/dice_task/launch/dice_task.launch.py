import launch
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the MindVision camera node.
    The dice_roller node is run separately (Terminal 2) so the user can
    trigger the task manually after confirming the camera is streaming.
    """

    camera_node = Node(
        package='dice_task',
        executable='mv_camera_node',
        name='mv_camera_node',
        respawn=True,
        respawn_delay=4,
    )

    return launch.LaunchDescription([
        camera_node,
    ])
