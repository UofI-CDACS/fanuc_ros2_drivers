"""
Launch file for the dice inspection task.

Usage (after filling in .env):
    ros2 launch dice_task dice_task.launch.py

ROBOT_NAME and ROBOT_IP are loaded automatically from the nearest .env file
(searched upward from the current working directory).

Individual values can still be overridden on the command line if needed:
    ros2 launch dice_task dice_task.launch.py robot_ip:=<IP> num_repetitions:=3
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


# ---------------------------------------------------------------------------
# Load .env — searches upward from cwd until a .env file is found
# ---------------------------------------------------------------------------

def _load_env_file():
    """Walk up from cwd looking for a .env file and load it into os.environ."""
    directory = os.getcwd()
    for _ in range(6):  # search up to 6 levels
        candidate = os.path.join(directory, '.env')
        if os.path.isfile(candidate):
            with open(candidate) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    key, _, val = line.partition('=')
                    # Only set if not already in the environment
                    os.environ.setdefault(key.strip(), val.strip())
            return
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent

_load_env_file()


# ---------------------------------------------------------------------------
# Resolve robot_name and robot_ip
# Priority: command-line arg > .env > default
# ---------------------------------------------------------------------------

robot_name = os.environ.get('ROBOT_NAME', 'bunsen')
robot_ip   = os.environ.get('ROBOT_IP', '')

cli_overrides = {}

for arg in sys.argv:
    if arg.startswith('robot_name:='):
        robot_name = arg.split(':=')[1]
    elif arg.startswith('robot_ip:='):
        robot_ip = arg.split(':=')[1]
    elif ':=' in arg:
        key, val = arg.split(':=', 1)
        try:
            cli_overrides[key] = int(val)
        except ValueError:
            try:
                cli_overrides[key] = float(val)
            except ValueError:
                cli_overrides[key] = val

cli_overrides['robot_name'] = robot_name


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():
    if not robot_ip:
        raise RuntimeError(
            "ROBOT_IP not set. Either fill in fanuc_ros2_drivers/.env "
            "or pass it on the command line: robot_ip:=<IP>"
        )

    config_file = os.path.join(
        get_package_share_directory('dice_task'),
        'config',
        'task_config.yaml',
    )

    # ── Robot driver nodes ────────────────────────────────────────────
    action_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('action_servers'), 'launch', 'action_servers.launch.py'
            ])
        ]),
        launch_arguments={'robot_name': robot_name, 'robot_ip': robot_ip}.items(),
    )

    message_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('msg_publishers'), 'launch', 'message_publishers.launch.py'
            ])
        ]),
        launch_arguments={'robot_name': robot_name, 'robot_ip': robot_ip}.items(),
    )

    srv_services = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('srv_services'), 'launch', 'srv_services.launch.py'
            ])
        ]),
        launch_arguments={'robot_name': robot_name, 'robot_ip': robot_ip}.items(),
    )

    # ── Dice task node ────────────────────────────────────────────────
    # YAML config is loaded first; CLI overrides take precedence.
    dice_task_node = Node(
        package='dice_task',
        executable='dice_task_node',
        name='dice_task_node',
        output='screen',
        parameters=[config_file, cli_overrides],
    )

    return LaunchDescription([
        action_servers,
        message_publishers,
        srv_services,
        dice_task_node,
    ])
