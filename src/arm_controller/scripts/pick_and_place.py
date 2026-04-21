#!/usr/bin/env python3
"""
Pick-and-place using MoveIt2 + Gz Link Attacher.

The arm moves to the object, attaches it via a runtime fixed joint,
carries it to the place location, and detaches.

Usage:
  ros2 run arm_controller pick_and_place.py

Requires:
  - Simulation running  (arm_env.launch.py)
  - MoveIt2 running     (move_group.launch.py)
  - Link attacher node  (link_attacher.launch.py)
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from linkattacher_msgs.srv import AttachLink, DetachLink


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # ---- Parameters (override from CLI if needed) ----
        self.declare_parameter('robot_model', 'robotic_arm')
        self.declare_parameter('robot_link', 'End_Effector')
        self.declare_parameter('object_model', 'package_box')
        self.declare_parameter('object_link', 'base_link')

        # Pick pose: above the box (box spawns at x=0.0, y=0.3, z=0.3)
        self.declare_parameter('pick_position', [0.0, 0.3, 0.38])
        # Place pose: above the rack (rack at x=0.6, y=0.0, z=0.1)
        self.declare_parameter('place_position', [0.35, 0.0, 0.35])
        # Home joint state
        self.declare_parameter('home_joints', [0.0, 0.8501, -2.4461, 0.0, 0.0])

        self.robot_model = self.get_parameter('robot_model').value
        self.robot_link = self.get_parameter('robot_link').value
        self.object_model = self.get_parameter('object_model').value
        self.object_link = self.get_parameter('object_link').value
        self.pick_pos = list(self.get_parameter('pick_position').value)
        self.place_pos = list(self.get_parameter('place_position').value)
        self.home_joints = list(self.get_parameter('home_joints').value)

        # ---- MoveIt2 ----
        cb_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=['l1', 'l2', 'l3', 'l4', 'ee'],
            base_link_name='base_link',
            end_effector_name='tool0',
            group_name='arm',
            callback_group=cb_group,
        )
        self.moveit2.allowed_planning_time = 5.0
        self.moveit2.num_planning_attempts = 20

        # ---- Link attacher service clients ----
        self.attach_client = self.create_client(
            AttachLink, '/link_attacher/attach')
        self.detach_client = self.create_client(
            DetachLink, '/link_attacher/detach')

    # ------------------------------------------------------------------
    def wait_for_services(self, timeout=10.0):
        self.get_logger().info('Waiting for link attacher services...')
        if not self.attach_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('/link_attacher/attach not available')
            return False
        if not self.detach_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('/link_attacher/detach not available')
            return False
        self.get_logger().info('Link attacher services ready.')
        return True

    # ------------------------------------------------------------------
    def wait_for_joint_state(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            js = self.moveit2.joint_state
            if js is not None:
                return js
            self.get_logger().info('Waiting for /joint_states...')
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.2)
        return None

    # ------------------------------------------------------------------
    def call_attach(self):
        req = AttachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name = self.robot_link
        req.model2_name = self.object_model
        req.link2_name = self.object_link
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() and future.result().success:
            self.get_logger().info('Object ATTACHED')
            return True
        self.get_logger().error('Attach failed')
        return False

    def call_detach(self):
        req = DetachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name = self.robot_link
        req.model2_name = self.object_model
        req.link2_name = self.object_link
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() and future.result().success:
            self.get_logger().info('Object DETACHED')
            return True
        self.get_logger().error('Detach failed')
        return False

    # ------------------------------------------------------------------
    def move_to_position(self, position, label='target'):
        """Plan and execute a position-only goal (orientation unconstrained)."""
        self.get_logger().info(f'Moving to {label}: {position}')
        start = self.moveit2.joint_state

        trajectory = self.moveit2.plan(
            position=position,
            frame_id='base_link',
            target_link='tool0',
            tolerance_position=0.01,
            tolerance_orientation=3.14,
            weight_position=1.0,
            weight_orientation=0.0,
            start_joint_state=start,
        )
        if trajectory is None:
            self.get_logger().error(f'Planning to {label} FAILED')
            return False

        self.moveit2.execute(trajectory)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'Reached {label}')
        return True

    def move_to_joints(self, joint_positions, label='joint target'):
        """Plan and execute a joint-space goal."""
        self.get_logger().info(f'Moving to {label}: {joint_positions}')
        start = self.moveit2.joint_state
        trajectory = self.moveit2.plan(
            joint_positions=joint_positions,
            start_joint_state=start,
        )
        if trajectory is None:
            self.get_logger().error(f'Joint planning to {label} FAILED')
            return False

        self.moveit2.execute(trajectory)
        self.moveit2.wait_until_executed()
        self.get_logger().info(f'Reached {label}')
        return True

    # ------------------------------------------------------------------
    def run(self):
        if not self.wait_for_services():
            return
        if self.wait_for_joint_state() is None:
            self.get_logger().error('No joint states received.')
            return

        self.get_logger().info('===== PICK AND PLACE START =====')

        # 1) Move to home
        self.move_to_joints(self.home_joints, 'HOME')
        time.sleep(1.0)

        # 2) Move above object (pre-pick)
        pre_pick = list(self.pick_pos)
        pre_pick[2] += 0.08  # 8 cm above pick
        self.move_to_position(pre_pick, 'PRE-PICK')
        time.sleep(0.5)

        # 3) Move to pick position
        self.move_to_position(self.pick_pos, 'PICK')
        time.sleep(0.5)

        # 4) Attach object
        self.call_attach()
        time.sleep(0.5)

        # 5) Lift
        lift = list(self.pick_pos)
        lift[2] += 0.12
        self.move_to_position(lift, 'LIFT')
        time.sleep(0.5)

        # 6) Move to place position
        self.move_to_position(self.place_pos, 'PLACE')
        time.sleep(0.5)

        # 7) Detach object
        self.call_detach()
        time.sleep(0.5)

        # 8) Retreat and go home
        retreat = list(self.place_pos)
        retreat[2] += 0.10
        self.move_to_position(retreat, 'RETREAT')
        self.move_to_joints(self.home_joints, 'HOME')

        self.get_logger().info('===== PICK AND PLACE COMPLETE =====')


def main():
    rclpy.init()
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
