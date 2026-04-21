#!/usr/bin/env python3
"""
Example of moving the robotic_arm to a pose goal using pymoveit2.
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from moveit_msgs.msg import MoveItErrorCodes


def plan_and_execute(
    moveit2: MoveIt2,
    *,
    position,
    quat_xyzw,
    use_orientation_goal,
    frame_id,
    target_link,
    cartesian,
    tolerance_position,
    tolerance_orientation,
    weight_position,
    weight_orientation,
    start_joint_state,
) -> bool:
    plan_kwargs = dict(
        position=position,
        frame_id=frame_id,
        target_link=target_link,
        cartesian=cartesian,
        tolerance_position=tolerance_position,
        tolerance_orientation=tolerance_orientation,
        weight_position=weight_position,
        weight_orientation=weight_orientation,
        start_joint_state=start_joint_state,
    )
    if use_orientation_goal:
        plan_kwargs["quat_xyzw"] = quat_xyzw

    trajectory = moveit2.plan(**plan_kwargs)
    if trajectory is None:
        return False

    moveit2.execute(trajectory)
    moveit2.wait_until_executed()
    return True


def plan_and_execute_joint_goal(
    moveit2: MoveIt2,
    *,
    joint_goal,
    start_joint_state,
) -> bool:
    trajectory = moveit2.plan(
        joint_positions=joint_goal,
        start_joint_state=start_joint_state,
    )
    if trajectory is None:
        return False

    moveit2.execute(trajectory)
    moveit2.wait_until_executed()
    return True


def compute_ik_quiet(
    moveit2: MoveIt2,
    *,
    position,
    quat_xyzw,
    ik_link_name,
    start_joint_state,
):
    future = moveit2.compute_ik_async(
        position=position,
        quat_xyzw=quat_xyzw,
        ik_link_name=ik_link_name,
        start_joint_state=start_joint_state,
    )
    if future is None:
        return None
    while not future.done():
        rclpy.spin_once(moveit2._node, timeout_sec=0.1)

    response = future.result()
    if response is None:
        return None
    if response.error_code.val != MoveItErrorCodes.SUCCESS:
        return None
    return response.solution.joint_state


def extract_joint_goal(joint_state_msg, joint_names):
    positions_by_name = {
        name: position
        for name, position in zip(joint_state_msg.name, joint_state_msg.position)
    }
    if any(name not in positions_by_name for name in joint_names):
        return None
    return [positions_by_name[name] for name in joint_names]


def generate_position_candidates(position, step_m, levels):
    base = [float(position[0]), float(position[1]), float(position[2])]
    candidates = [base]
    # Fast axis-aligned sweep around the requested point.
    # This avoids an expensive full 3D grid while still finding nearby reachable poses.
    for i in range(1, levels + 1):
        delta = i * step_m
        candidates.extend(
            [
                [base[0] + delta, base[1], base[2]],
                [base[0] - delta, base[1], base[2]],
                [base[0], base[1] + delta, base[2]],
                [base[0], base[1] - delta, base[2]],
                [base[0], base[1], base[2] + delta],
                [base[0], base[1], base[2] - delta],
            ]
        )
    return candidates


def try_ik_seeded_joint_planning(
    moveit2: MoveIt2,
    *,
    node: Node,
    start_joint_state,
    target_link,
    position_candidates,
    quat_candidates,
    success_log_prefix: str,
) -> bool:
    for candidate_position in position_candidates:
        for candidate_quat in quat_candidates:
            ik_joint_state = compute_ik_quiet(
                position=candidate_position,
                quat_xyzw=candidate_quat,
                ik_link_name=target_link,
                start_joint_state=start_joint_state,
                moveit2=moveit2,
            )
            if ik_joint_state is None:
                continue

            joint_goal = extract_joint_goal(ik_joint_state, moveit2.joint_names)
            if joint_goal is None:
                node.get_logger().warning(
                    "IK returned unexpected joint names; skipping candidate."
                )
                continue

            success = plan_and_execute_joint_goal(
                moveit2,
                joint_goal=joint_goal,
                start_joint_state=start_joint_state,
            )
            if success:
                node.get_logger().info(
                    f"{success_log_prefix} position={candidate_position} "
                    f"quat_xyzw={candidate_quat}"
                )
                return True

    return False


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for pose + how strictly to enforce orientation
    # Default position is a safe pose above the table; orientation is relaxed by default
    node.declare_parameter("position", [0.0, 0.25, 0.35])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
    node.declare_parameter("cartesian", False)
    node.declare_parameter("constrain_orientation", False)
    node.declare_parameter("tolerance_position", 0.01)        # meters
    node.declare_parameter("tolerance_orientation", 0.5)       # radians
    node.declare_parameter("weight_position", 1.0)
    node.declare_parameter("weight_orientation", 1.0)
    node.declare_parameter("frame_id", "base_link")
    node.declare_parameter("target_link", "tool0")
    node.declare_parameter("auto_relax_orientation_on_failure", True)
    node.declare_parameter("joint_state_wait_timeout_sec", 5.0)
    node.declare_parameter("allowed_planning_time_sec", 2.0)
    node.declare_parameter("num_planning_attempts", 20)
    node.declare_parameter("use_ik_seeded_joint_planning", True)
    node.declare_parameter("search_nearby_on_ik_failure", False)
    node.declare_parameter("nearby_search_step_m", 0.02)
    node.declare_parameter("nearby_search_levels", 1)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=['l1', 'l2', 'l3', 'l4', 'ee'],
        base_link_name='base_link',
        end_effector_name='tool0',
        group_name='arm',
        callback_group=callback_group,
    )

    try:
        # Get parameters
        position = node.get_parameter("position").get_parameter_value().double_array_value
        quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
        cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
        constrain_orientation = node.get_parameter("constrain_orientation").get_parameter_value().bool_value
        tol_pos = node.get_parameter("tolerance_position").get_parameter_value().double_value
        tol_ori = node.get_parameter("tolerance_orientation").get_parameter_value().double_value
        weight_pos = node.get_parameter("weight_position").get_parameter_value().double_value
        weight_ori = node.get_parameter("weight_orientation").get_parameter_value().double_value
        frame_id = node.get_parameter("frame_id").get_parameter_value().string_value
        target_link = node.get_parameter("target_link").get_parameter_value().string_value
        auto_relax = node.get_parameter("auto_relax_orientation_on_failure").get_parameter_value().bool_value
        joint_state_wait_timeout = node.get_parameter("joint_state_wait_timeout_sec").get_parameter_value().double_value
        allowed_planning_time = node.get_parameter("allowed_planning_time_sec").get_parameter_value().double_value
        num_planning_attempts = node.get_parameter("num_planning_attempts").get_parameter_value().integer_value
        use_ik_seeded_joint_planning = node.get_parameter("use_ik_seeded_joint_planning").get_parameter_value().bool_value
        search_nearby = node.get_parameter("search_nearby_on_ik_failure").get_parameter_value().bool_value
        nearby_step_m = node.get_parameter("nearby_search_step_m").get_parameter_value().double_value
        nearby_levels = node.get_parameter("nearby_search_levels").get_parameter_value().integer_value

        # If the arm has fewer than 6 DOF, enforcing orientation can make many poses unsolvable
        # so allow disabling it (default).
        if not constrain_orientation:
            weight_ori = 0.0
            tol_ori = 3.14159  # effectively free orientation
            if use_ik_seeded_joint_planning:
                # IK seeding uses explicit quaternion candidates, which can accidentally
                # over-constrain "position-only" requests. For unconstrained orientation,
                # prefer direct position planning (no orientation goal in the request).
                use_ik_seeded_joint_planning = False
                node.get_logger().info(
                    "Orientation is unconstrained; using direct position planning "
                    "instead of IK-seeded joint planning."
                )

        # MoveIt2 defaults are conservative (0.5s); increase by default for harder goals.
        if allowed_planning_time > 0.0:
            moveit2.allowed_planning_time = float(allowed_planning_time)
        if num_planning_attempts > 0:
            moveit2.num_planning_attempts = int(num_planning_attempts)

        # Move to pose
        node.get_logger().info(
            f"Moving to position={list(position)} quat_xyzw={list(quat_xyzw)} "
            f"frame_id={frame_id} target_link={target_link} cartesian={cartesian} "
            f"constrain_orientation={constrain_orientation} "
            f"planning_time={moveit2.allowed_planning_time}s attempts={moveit2.num_planning_attempts} "
            f"tol_pos={tol_pos} tol_ori={tol_ori}"
        )

        # Wait once for a current state and then pass it explicitly to avoid repetitive warnings.
        start_joint_state = None
        wait_deadline = time.monotonic() + max(0.1, joint_state_wait_timeout)
        while time.monotonic() < wait_deadline:
            start_joint_state = moveit2.joint_state
            if start_joint_state is not None:
                break
            node.get_logger().warning("Waiting for /joint_states...")
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.2)
        if start_joint_state is None:
            node.get_logger().error(
                "No /joint_states received. Ensure joint_state_broadcaster is active."
            )
            return

        success = False
        if use_ik_seeded_joint_planning:
            # Build IK orientation candidates. If orientation is constrained, use only the requested one.
            ik_quat_candidates = [list(quat_xyzw)]
            if auto_relax:
                if constrain_orientation:
                    node.get_logger().warning(
                        "Strict orientation was requested. "
                        "Will try relaxed orientation fallbacks if needed."
                    )
                ik_quat_candidates.extend(
                    [
                        [0.0, 0.70710678, 0.0, 0.70710678],
                        [0.70710678, 0.0, 0.0, 0.70710678],
                        [0.0, 0.0, 0.70710678, 0.70710678],
                        [0.0, -0.70710678, 0.0, 0.70710678],
                        [-0.70710678, 0.0, 0.0, 0.70710678],
                        [0.0, 0.0, -0.70710678, 0.70710678],
                    ]
                )

            position_candidates = [list(position)]
            if search_nearby and nearby_step_m > 0.0 and nearby_levels > 0:
                position_candidates = generate_position_candidates(
                    position=position,
                    step_m=nearby_step_m,
                    levels=nearby_levels,
                )

            success = try_ik_seeded_joint_planning(
                moveit2,
                node=node,
                start_joint_state=start_joint_state,
                target_link=target_link,
                position_candidates=position_candidates,
                quat_candidates=ik_quat_candidates,
                success_log_prefix="IK-seeded plan succeeded with",
            )
        else:
            success = plan_and_execute(
                moveit2,
                position=position,
                quat_xyzw=quat_xyzw,
                use_orientation_goal=constrain_orientation,
                frame_id=frame_id,
                target_link=target_link,
                cartesian=cartesian,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_ori,
                weight_position=weight_pos,
                weight_orientation=weight_ori,
                start_joint_state=start_joint_state,
            )
            if (
                not success
                and search_nearby
                and nearby_step_m > 0.0
                and nearby_levels > 0
            ):
                node.get_logger().warning(
                    "Direct position planning failed at requested point. "
                    "Trying nearby position fallbacks."
                )
                for candidate_position in generate_position_candidates(
                    position=position,
                    step_m=nearby_step_m,
                    levels=nearby_levels,
                )[1:]:
                    success = plan_and_execute(
                        moveit2,
                        position=candidate_position,
                        quat_xyzw=quat_xyzw,
                        use_orientation_goal=constrain_orientation,
                        frame_id=frame_id,
                        target_link=target_link,
                        cartesian=cartesian,
                        tolerance_position=tol_pos,
                        tolerance_orientation=tol_ori,
                        weight_position=weight_pos,
                        weight_orientation=weight_ori,
                        start_joint_state=start_joint_state,
                    )
                    if success:
                        node.get_logger().info(
                            "Direct plan succeeded with nearby "
                            f"position={candidate_position}"
                        )
                        break

            if not success and not constrain_orientation:
                # Last fallback for position-only requests:
                # sample a small set of orientations with IK and plan in joint space.
                node.get_logger().warning(
                    "Direct position planning failed. "
                    "Trying IK-seeded orientation fallbacks."
                )
                ik_quat_candidates = [
                    list(quat_xyzw),
                    [0.0, 0.70710678, 0.0, 0.70710678],
                    [0.70710678, 0.0, 0.0, 0.70710678],
                    [0.0, 0.0, 0.70710678, 0.70710678],
                    [0.0, -0.70710678, 0.0, 0.70710678],
                    [-0.70710678, 0.0, 0.0, 0.70710678],
                    [0.0, 0.0, -0.70710678, 0.70710678],
                ]
                ik_position_candidates = [list(position)]
                if search_nearby and nearby_step_m > 0.0 and nearby_levels > 0:
                    ik_position_candidates = generate_position_candidates(
                        position=position,
                        step_m=nearby_step_m,
                        levels=nearby_levels,
                    )

                success = try_ik_seeded_joint_planning(
                    moveit2,
                    node=node,
                    start_joint_state=start_joint_state,
                    target_link=target_link,
                    position_candidates=ik_position_candidates,
                    quat_candidates=ik_quat_candidates,
                    success_log_prefix="IK fallback succeeded with",
                )

        if not success:
            node.get_logger().error(
                "Planning failed for this pose. "
                "Try constrain_orientation:=false or a different target position/quaternion."
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
