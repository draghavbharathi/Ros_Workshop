#!/usr/bin/env python3

import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class SpawnAGV(Node):
    def __init__(self):
        super().__init__('spawn_agv')

        pkg_share = get_package_share_directory('agv')
        pkg_share_parent = os.path.dirname(pkg_share)
        urdf_file = os.path.join(pkg_share, 'urdf', 'AGV.urdf')

        if not os.path.isfile(urdf_file):
            self.get_logger().error(f'URDF not found: {urdf_file}')
            sys.exit(1)

        world_name = 'robotic_arm_world'
        entity_name = 'agv'
        x, y, z = '0.0', '0.0', '0.02'

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', world_name,
            '-name', entity_name,
            '-allow_renaming', 'true',
            '-file', urdf_file,
            '-x', x,
            '-y', y,
            '-z', z,
        ]

        self.get_logger().info(
            f'Spawning "{entity_name}" into world "{world_name}" at ({x}, {y}, {z})'
        )

        gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        resource_entries = [pkg_share_parent]
        if gz_resource_path:
            resource_entries.append(gz_resource_path)

        env = os.environ.copy()
        env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(resource_entries)

        result = subprocess.run(cmd, env=env)
        if result.returncode != 0:
            self.get_logger().error(
                f'Spawn command failed with exit code {result.returncode}'
            )
            sys.exit(result.returncode)

        self.get_logger().info('AGV spawned successfully. Shutting down.')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnAGV()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
