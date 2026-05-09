"""
Launch file for a single robot's task node (dice → camera → conveyor).

For a two-robot setup, launch this file once per robot with different
robot_name, robot_ip, and robot_index arguments.  The modbus_server is
launched separately (once, shared by both robots):

    Terminal 1:  ros2 launch modbus_server modbus_server.launch.py
    Terminal 2:  ros2 launch robot_task task.launch.py robot_index:=1
    Terminal 3:  ros2 launch robot_task task.launch.py robot_index:=2

ROBOT_NAME and ROBOT_IP are resolved in priority order:
  1. Command-line argument  (robot_name:= / robot_ip:=)
  2. .env file              (searched: dotenv:=<path> arg, then cwd/.env)
  3. Environment variable   ROBOT_NAME / ROBOT_IP
  4. Defaults               robot_name='bunsen'; robot_ip raises if unset.

.env file format (workspace root):
    ROBOT_NAME=bunsen
    ROBOT_IP=192.168.0.100
    # lines starting with # are comments

Additional node parameters can be overridden on the command line, e.g.:
    robot_index:=2 max_retries:=6
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
# Load .env file (if present) before reading env vars
# ---------------------------------------------------------------------------

def _load_dotenv(path: str):
    """Parse a simple KEY=VALUE .env file into os.environ (no-op if missing)."""
    if not os.path.isfile(path):
        return
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, _, val = line.partition('=')
            os.environ.setdefault(key.strip(), val.strip())


# Allow dotenv:=<path> to override the default location (cwd/.env)
_dotenv_path = os.path.join(os.getcwd(), '.env')
for _arg in sys.argv:
    if _arg.startswith('dotenv:='):
        _dotenv_path = _arg.split(':=', 1)[1]
        break

_load_dotenv(_dotenv_path)

# dice_vision lives alongside the workspace .env — add it to PYTHONPATH so
# the task node can import it without sourcing setup_ws.bash.
_ws_root    = os.path.dirname(os.path.abspath(_dotenv_path))
_fanuc_root = os.path.join(_ws_root, 'fanuc_ros2_drivers')
_extra_pythonpath = _fanuc_root + ':' + os.environ.get('PYTHONPATH', '')


# ---------------------------------------------------------------------------
# Resolve robot_name, robot_ip, and any extra CLI parameter overrides
# ---------------------------------------------------------------------------

robot_name  = os.environ.get('ROBOT_NAME', 'bunsen')
robot_ip    = os.environ.get('ROBOT_IP', '')
robot_index = int(os.environ.get('ROBOT_INDEX', '1'))

cli_overrides = {}

for arg in sys.argv:
    if arg.startswith('robot_name:='):
        robot_name = arg.split(':=', 1)[1]
    elif arg.startswith('robot_ip:='):
        robot_ip = arg.split(':=', 1)[1]
    elif arg.startswith('robot_index:='):
        try:
            robot_index = int(arg.split(':=', 1)[1])
        except ValueError:
            pass
    elif ':=' in arg:
        key, val = arg.split(':=', 1)
        try:
            cli_overrides[key] = int(val)
        except ValueError:
            try:
                cli_overrides[key] = float(val)
            except ValueError:
                cli_overrides[key] = val

cli_overrides['robot_name']  = robot_name
cli_overrides['robot_index'] = robot_index


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------

def generate_launch_description():
    if not robot_ip:
        raise RuntimeError(
            "ROBOT_IP not set. Pass it on the command line: robot_ip:=<IP>"
        )

    config_file = os.path.join(
        get_package_share_directory('robot_task'),
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

    # ── Task node ─────────────────────────────────────────────────────
    # Node name matches the YAML section key so each robot picks up its own
    # calibrated parameters from task_config.yaml automatically.
    task_node = Node(
        package='robot_task',
        executable='task_node',
        name=f'robot{robot_index}_task_node',
        output='screen',
        parameters=[config_file, cli_overrides],
        additional_env={'PYTHONPATH': _extra_pythonpath},
    )

    return LaunchDescription([
        action_servers,
        message_publishers,
        srv_services,
        task_node,
    ])
