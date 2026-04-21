#!/usr/bin/env python3
"""
ROS 2 node that bridges AttachLink / DetachLink ROS services
to the gz-transport services exposed by GzLinkAttacherSystem.
"""

import subprocess
import rclpy
from rclpy.node import Node
from linkattacher_msgs.srv import AttachLink, DetachLink


class LinkAttacherServiceNode(Node):
    def __init__(self):
        super().__init__('link_attacher_service_node')

        self.attach_srv = self.create_service(
            AttachLink, '/link_attacher/attach', self._attach_cb)
        self.detach_srv = self.create_service(
            DetachLink, '/link_attacher/detach', self._detach_cb)

        self.get_logger().info(
            'Link attacher ROS 2 bridge ready '
            '(/link_attacher/attach, /link_attacher/detach)')

    # ------------------------------------------------------------------
    def _call_gz_service(self, service_name: str,
                         model1: str, link1: str,
                         model2: str, link2: str) -> bool:
        """Call the gz-transport service via the `gz service` CLI."""
        req_str = (
            f'data: "{model1}" '
            f'data: "{link1}" '
            f'data: "{model2}" '
            f'data: "{link2}"'
        )
        cmd = [
            'gz', 'service',
            '-s', service_name,
            '--reqtype', 'gz.msgs.StringMsg_V',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req_str,
        ]
        self.get_logger().info(f'Calling: {" ".join(cmd)}')
        try:
            result = subprocess.run(
                cmd, capture_output=True, text=True, timeout=10)
            self.get_logger().info(f'gz service stdout: {result.stdout.strip()}')
            if result.returncode != 0:
                self.get_logger().error(f'gz service stderr: {result.stderr.strip()}')
                return False
            return 'data: true' in result.stdout.lower()
        except Exception as e:
            self.get_logger().error(f'gz service call failed: {e}')
            return False

    # ------------------------------------------------------------------
    def _attach_cb(self, request: AttachLink.Request,
                   response: AttachLink.Response):
        self.get_logger().info(
            f'Attach request: {request.model1_name}::{request.link1_name} '
            f'<-> {request.model2_name}::{request.link2_name}')
        response.success = self._call_gz_service(
            '/link_attacher/attach',
            request.model1_name, request.link1_name,
            request.model2_name, request.link2_name)
        return response

    def _detach_cb(self, request: DetachLink.Request,
                   response: DetachLink.Response):
        self.get_logger().info(
            f'Detach request: {request.model1_name}::{request.link1_name} '
            f'<-> {request.model2_name}::{request.link2_name}')
        response.success = self._call_gz_service(
            '/link_attacher/detach',
            request.model1_name, request.link1_name,
            request.model2_name, request.link2_name)
        return response


def main():
    rclpy.init()
    node = LinkAttacherServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
