"""
Entry point — selects Robot1TaskNode or Robot2TaskNode based on robot_index arg.

Usage:
    ros2 run robot_task task_node --ros-args -p robot_index:=1
    ros2 run robot_task task_node --ros-args -p robot_index:=2

Or via launch file (recommended):
    ros2 launch robot_task task.launch.py robot_index:=1
"""

import asyncio
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from robot_task.robot1_node import Robot1TaskNode
from robot_task.robot2_node import Robot2TaskNode


def _peek_robot_index() -> int:
    """
    Resolve robot_index before node creation.

    Scans ALL --params-file entries and returns the LAST robot_index found.
    The launch file passes [task_config.yaml, cli_overrides_tempfile] in order,
    so the cli_overrides value (robot_index:=N from the command line) always
    wins by appearing last.  The temp file uses '/**' as the node key.
    """
    import yaml

    last_index = None

    for i, arg in enumerate(sys.argv):
        if arg == '--params-file' and i + 1 < len(sys.argv):
            try:
                with open(sys.argv[i + 1]) as f:
                    params = yaml.safe_load(f) or {}
                for section in params.values():
                    if isinstance(section, dict):
                        idx = section.get('ros__parameters', {}).get('robot_index')
                        if idx is not None:
                            last_index = int(idx)
            except Exception:
                pass

    return last_index if last_index is not None else 1


def main(args=None):
    rclpy.init(args=args)

    robot_index = _peek_robot_index()
    node = Robot1TaskNode() if robot_index == 1 else Robot2TaskNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        asyncio.run(node.execute_task())
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node._shutdown_event.set()
        node._modbus.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
