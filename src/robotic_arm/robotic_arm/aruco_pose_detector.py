#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


ARUCO_DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def create_detector(dictionary_name: str, detect_inverted_marker: bool):
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_MAP[dictionary_name])
    # Prefer DetectorParameters_create() because some OpenCV builds crash
    # with DetectorParameters() when detectMarkers() is called.
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        parameters = cv2.aruco.DetectorParameters_create()
    elif hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        raise RuntimeError("OpenCV aruco detector parameters API not found.")
    if hasattr(parameters, "detectInvertedMarker"):
        parameters.detectInvertedMarker = bool(detect_inverted_marker)
    detector = None
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    return dictionary, parameters, detector


def detect_markers(detector, dictionary, parameters, gray):
    if detector is not None:
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def marker_object_points(marker_length: float) -> np.ndarray:
    half = marker_length * 0.5
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


class ArucoPoseDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_pose_detector")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("marker_length", 0.15)
        self.declare_parameter("aruco_dict", "DICT_4X4_1000")
        self.declare_parameter("print_rate_hz", 2.0)
        self.declare_parameter("window_name", "ArUco Pose Preview")
        self.declare_parameter("show_window", False)
        self.declare_parameter("preview_topic", "/aruco/preview")
        self.declare_parameter("detect_inverted_marker", False)
        self.declare_parameter("poses_topic", "/aruco/poses")
        self.declare_parameter("marker_ids_topic", "/aruco/ids")
        self.declare_parameter("target_pose_topic", "/aruco/target_pose")
        self.declare_parameter("target_marker_id", -1)
        # When true, target_pose is published in base_frame_id instead of camera frame.
        self.declare_parameter("publish_target_pose_in_base_frame", False)
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("tf_child_frame_prefix", "aruco_marker_")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_base_frame_tf", True)
        self.declare_parameter("base_tf_child_frame_prefix", "aruco_box_")
        self.declare_parameter("tf_lookup_timeout_sec", 0.05)
        self.declare_parameter("pose_z_rotation_deg", -90.0)
        self.declare_parameter("pose_solver", "solvepnp")
        self.declare_parameter("draw_axes", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.marker_length = float(self.get_parameter("marker_length").value)
        self.dict_name = str(self.get_parameter("aruco_dict").value)
        self.print_rate_hz = float(self.get_parameter("print_rate_hz").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.preview_topic = str(self.get_parameter("preview_topic").value)
        self.detect_inverted_marker = bool(
            self.get_parameter("detect_inverted_marker").value
        )
        self.poses_topic = str(self.get_parameter("poses_topic").value)
        self.marker_ids_topic = str(self.get_parameter("marker_ids_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.target_marker_id = int(self.get_parameter("target_marker_id").value)
        self.publish_target_pose_in_base_frame = bool(
            self.get_parameter("publish_target_pose_in_base_frame").value
        )
        self.output_frame_id = str(self.get_parameter("output_frame_id").value).strip()
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.tf_child_frame_prefix = str(
            self.get_parameter("tf_child_frame_prefix").value
        ).strip()
        self.base_frame_id = str(self.get_parameter("base_frame_id").value).strip()
        self.publish_base_frame_tf = bool(
            self.get_parameter("publish_base_frame_tf").value
        )
        self.base_tf_child_frame_prefix = str(
            self.get_parameter("base_tf_child_frame_prefix").value
        ).strip()
        self.tf_lookup_timeout_sec = float(
            self.get_parameter("tf_lookup_timeout_sec").value
        )
        self.pose_z_rotation_deg = float(
            self.get_parameter("pose_z_rotation_deg").value
        )
        self.pose_solver = str(self.get_parameter("pose_solver").value).strip().lower()
        self.draw_axes = bool(self.get_parameter("draw_axes").value)

        if self.marker_length <= 0.0:
            raise ValueError("Parameter marker_length must be > 0.0")
        if self.dict_name not in ARUCO_DICT_MAP:
            raise ValueError(f"Unsupported aruco_dict: {self.dict_name}")
        if self.print_rate_hz <= 0.0:
            self.print_rate_hz = 2.0
        if self.target_marker_id < -1:
            raise ValueError("Parameter target_marker_id must be -1 or >= 0")
        if self.pose_solver not in {"solvepnp", "aruco"}:
            raise ValueError("Parameter pose_solver must be one of: solvepnp, aruco")
        if not self.tf_child_frame_prefix:
            self.tf_child_frame_prefix = "aruco_marker_"
        if self.tf_lookup_timeout_sec < 0.0:
            self.tf_lookup_timeout_sec = 0.0
        if not self.base_tf_child_frame_prefix:
            self.base_tf_child_frame_prefix = "aruco_box_"
        self.pose_z_rotation_rad = np.deg2rad(self.pose_z_rotation_deg)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_points = marker_object_points(self.marker_length)
        self.got_camera_info = False
        self.warned_empty_frame_for_tf = False
        self.last_log_time = 0.0
        self.min_log_dt = 1.0 / self.print_rate_hz
        self.last_tf_warn_time = 0.0

        self.dictionary, self.parameters, self.detector = create_detector(
            self.dict_name, self.detect_inverted_marker
        )
        self.preview_pub = self.create_publisher(Image, self.preview_topic, 10)
        self.poses_pub = self.create_publisher(PoseArray, self.poses_topic, 10)
        self.marker_ids_pub = self.create_publisher(
            Int32MultiArray, self.marker_ids_topic, 10
        )
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Subscribed image topic: {self.image_topic}, "
            f"camera info topic: {self.camera_info_topic}"
        )
        self.get_logger().info(f"Publishing annotated preview on: {self.preview_topic}")
        self.get_logger().info(
            f"Publishing marker poses on: {self.poses_topic}, ids on: {self.marker_ids_topic}"
        )
        self.get_logger().info(
            f"ArUco dict: {self.dict_name}, detect_inverted_marker: "
            f"{str(self.detect_inverted_marker).lower()}"
        )
        self.get_logger().info(
            f"Pose solver: {self.pose_solver}, draw_axes: {str(self.draw_axes).lower()}"
        )
        self.get_logger().info(
            f"TF publish: {str(self.publish_tf).lower()}, child prefix: "
            f"{self.tf_child_frame_prefix}"
        )
        self.get_logger().info(
            f"Base-relative TF publish: {str(self.publish_base_frame_tf).lower()}, "
            f"base frame: {self.base_frame_id}, child prefix: {self.base_tf_child_frame_prefix}"
        )
        self.get_logger().info(
            f"Pose local Z rotation offset: {self.pose_z_rotation_deg:.2f} deg"
        )
        if self.target_marker_id >= 0:
            self.get_logger().info(
                f"Publishing target marker {self.target_marker_id} pose on: "
                f"{self.target_pose_topic}"
            )
            self.get_logger().info(
                "Target pose frame mode: "
                + (
                    f"{self.base_frame_id} (base-relative)"
                    if self.publish_target_pose_in_base_frame and self.base_frame_id
                    else "detector frame"
                )
            )
        if self.show_window:
            self.get_logger().info("Press 'q' in preview window to stop.")
        else:
            self.get_logger().info("OpenCV window disabled (show_window:=false).")

    @staticmethod
    def image_msg_to_bgr(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding not in {"bgr8", "rgb8", "bgra8", "rgba8", "mono8"}:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        channels = 1 if encoding == "mono8" else (4 if "a8" in encoding else 3)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if channels == 1:
            min_size = msg.height * msg.step
            if raw.size < min_size:
                raise ValueError("Image buffer too small for mono image.")
            if msg.step == msg.width:
                frame = raw[: msg.height * msg.width].reshape(msg.height, msg.width)
            else:
                frame = raw[:min_size].reshape(msg.height, msg.step)[:, : msg.width]
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        expected_step = msg.width * channels
        min_size = msg.height * msg.step
        if raw.size < min_size:
            raise ValueError("Image buffer too small for color image.")
        if msg.step < expected_step:
            raise ValueError("Invalid step size in image message.")

        if msg.step == expected_step:
            frame = raw[: msg.height * expected_step].reshape(msg.height, msg.width, channels)
        else:
            padded = raw[:min_size].reshape(msg.height, msg.step)
            frame = padded[:, :expected_step].reshape(msg.height, msg.width, channels)

        if encoding == "bgr8":
            return frame
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

    @staticmethod
    def bgr_to_image_msg(frame: np.ndarray, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * 3)
        msg.data = frame.tobytes()
        return msg

    @staticmethod
    def rvec_to_quaternion(rvec: np.ndarray) -> tuple[float, float, float, float]:
        rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])

        if trace > 0.0:
            s = max(np.sqrt(trace + 1.0) * 2.0, 1e-12)
            qw = 0.25 * s
            qx = (rot[2, 1] - rot[1, 2]) / s
            qy = (rot[0, 2] - rot[2, 0]) / s
            qz = (rot[1, 0] - rot[0, 1]) / s
        elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
            s = max(np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0, 1e-12)
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif rot[1, 1] > rot[2, 2]:
            s = max(np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0, 1e-12)
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = max(np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0, 1e-12)
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s

        norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        return qx / norm, qy / norm, qz / norm, qw / norm

    @classmethod
    def pose_from_rvec_tvec(cls, rvec: np.ndarray, tvec: np.ndarray) -> Pose:
        tvec_flat = np.asarray(tvec, dtype=np.float64).reshape(-1)
        qx, qy, qz, qw = cls.rvec_to_quaternion(rvec)

        pose = Pose()
        pose.position.x = float(tvec_flat[0])
        pose.position.y = float(tvec_flat[1])
        pose.position.z = float(tvec_flat[2])
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    @staticmethod
    def quaternion_to_rotation_matrix(
        qx: float, qy: float, qz: float, qw: float
    ) -> np.ndarray:
        q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
        norm = float(np.linalg.norm(q))
        if norm <= 1e-12:
            return np.eye(3, dtype=np.float64)
        qx, qy, qz, qw = q / norm

        return np.array(
            [
                [
                    1.0 - 2.0 * (qy * qy + qz * qz),
                    2.0 * (qx * qy - qz * qw),
                    2.0 * (qx * qz + qy * qw),
                ],
                [
                    2.0 * (qx * qy + qz * qw),
                    1.0 - 2.0 * (qx * qx + qz * qz),
                    2.0 * (qy * qz - qx * qw),
                ],
                [
                    2.0 * (qx * qz - qy * qw),
                    2.0 * (qy * qz + qx * qw),
                    1.0 - 2.0 * (qx * qx + qy * qy),
                ],
            ],
            dtype=np.float64,
        )

    @classmethod
    def rotation_matrix_to_quaternion(
        cls, rotation_matrix: np.ndarray
    ) -> tuple[float, float, float, float]:
        rvec, _ = cv2.Rodrigues(np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3))
        return cls.rvec_to_quaternion(rvec)

    @classmethod
    def transform_pose(cls, parent_child_tf: TransformStamped, pose_in_child: Pose) -> Pose:
        tf_t = parent_child_tf.transform.translation
        tf_q = parent_child_tf.transform.rotation
        rot_parent_child = cls.quaternion_to_rotation_matrix(tf_q.x, tf_q.y, tf_q.z, tf_q.w)
        trans_parent_child = np.array([tf_t.x, tf_t.y, tf_t.z], dtype=np.float64)

        pose_t = pose_in_child.position
        pose_q = pose_in_child.orientation
        rot_child_pose = cls.quaternion_to_rotation_matrix(
            pose_q.x, pose_q.y, pose_q.z, pose_q.w
        )
        trans_child_pose = np.array([pose_t.x, pose_t.y, pose_t.z], dtype=np.float64)

        rot_parent_pose = rot_parent_child @ rot_child_pose
        trans_parent_pose = trans_parent_child + rot_parent_child @ trans_child_pose
        qx, qy, qz, qw = cls.rotation_matrix_to_quaternion(rot_parent_pose)

        pose = Pose()
        pose.position.x = float(trans_parent_pose[0])
        pose.position.y = float(trans_parent_pose[1])
        pose.position.z = float(trans_parent_pose[2])
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    @classmethod
    def rotate_pose_about_local_z(cls, pose: Pose, angle_rad: float) -> Pose:
        if abs(angle_rad) <= 1e-12:
            return pose

        cz = float(np.cos(angle_rad))
        sz = float(np.sin(angle_rad))
        rot_z = np.array(
            [
                [cz, -sz, 0.0],
                [sz, cz, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        q = pose.orientation
        rot_pose = cls.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        rot_rotated = rot_pose @ rot_z
        qx, qy, qz, qw = cls.rotation_matrix_to_quaternion(rot_rotated)

        rotated_pose = Pose()
        rotated_pose.position.x = pose.position.x
        rotated_pose.position.y = pose.position.y
        rotated_pose.position.z = pose.position.z
        rotated_pose.orientation.x = float(qx)
        rotated_pose.orientation.y = float(qy)
        rotated_pose.orientation.z = float(qz)
        rotated_pose.orientation.w = float(qw)
        return rotated_pose

    def lookup_base_from_frame(self, frame_id: str, stamp) -> TransformStamped | None:
        if not frame_id or not self.base_frame_id:
            return None

        timeout = Duration(seconds=self.tf_lookup_timeout_sec)
        stamp_time = Time.from_msg(stamp)

        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame_id,
                frame_id,
                stamp_time,
                timeout=timeout,
            )
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(
                    self.base_frame_id,
                    frame_id,
                    Time(),
                    timeout=timeout,
                )
            except TransformException as exc:
                now = time.time()
                if now - self.last_tf_warn_time >= self.min_log_dt:
                    self.get_logger().warn(
                        f"Failed to lookup TF from '{frame_id}' to '{self.base_frame_id}': {exc}"
                    )
                    self.last_tf_warn_time = now
                return None

    def camera_info_cb(self, msg: CameraInfo) -> None:
        cam = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if cam[0, 0] <= 0.0 or cam[1, 1] <= 0.0:
            return

        dist = np.asarray(msg.d, dtype=np.float64)
        if dist.size == 0:
            dist = np.zeros((5, 1), dtype=np.float64)
        else:
            dist = dist.reshape(-1, 1)

        self.camera_matrix = np.ascontiguousarray(cam, dtype=np.float64)
        self.dist_coeffs = np.ascontiguousarray(dist, dtype=np.float64)

        if not self.got_camera_info:
            self.got_camera_info = True
            self.get_logger().info("Camera intrinsics received from camera_info.")

    def estimate_poses(self, corners):
        pose_estimates = [None] * len(corners)
        if self.pose_solver == "aruco":
            if not hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
                self.get_logger().warn(
                    "estimatePoseSingleMarkers unavailable; falling back to solvepnp."
                )
            else:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_length,
                    self.camera_matrix,
                    self.dist_coeffs,
                )
                for i in range(len(corners)):
                    pose_estimates[i] = (rvecs[i], tvecs[i])
                return pose_estimates

        pnp_flag = (
            cv2.SOLVEPNP_IPPE_SQUARE
            if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE")
            else cv2.SOLVEPNP_ITERATIVE
        )
        for i, marker_corners in enumerate(corners):
            img_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
            ok, rvec, tvec = cv2.solvePnP(
                self.marker_points,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=pnp_flag,
            )
            if ok:
                pose_estimates[i] = (rvec, tvec)
        return pose_estimates

    def image_cb(self, msg: Image) -> None:
        frame_id = self.output_frame_id if self.output_frame_id else msg.header.frame_id
        if self.publish_tf and not frame_id and not self.warned_empty_frame_for_tf:
            self.get_logger().warn(
                "Frame id is empty; skipping /tf publication. Set output_frame_id parameter."
            )
            self.warned_empty_frame_for_tf = True
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = msg.header.stamp
        pose_array_msg.header.frame_id = frame_id
        marker_ids_msg = Int32MultiArray()
        target_pose_msg = None
        tf_msgs = []

        try:
            frame = self.image_msg_to_bgr(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Image conversion error: {exc}")
            return

        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        try:
            corners, ids, _ = detect_markers(
                self.detector, self.dictionary, self.parameters, gray
            )
        except cv2.error as exc:
            self.get_logger().error(f"ArUco detect error: {exc}")
            self.poses_pub.publish(pose_array_msg)
            self.marker_ids_pub.publish(marker_ids_msg)
            self.preview_pub.publish(self.bgr_to_image_msg(frame, msg.header))
            return

        if ids is not None and len(ids) > 0:
            marker_ids_msg.data = [int(marker_id) for marker_id in ids.flatten()]
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if self.got_camera_info:
                try:
                    pose_estimates = self.estimate_poses(corners)
                except cv2.error as exc:
                    self.get_logger().error(f"ArUco pose error: {exc}")
                    self.poses_pub.publish(pose_array_msg)
                    self.marker_ids_pub.publish(marker_ids_msg)
                    self.preview_pub.publish(self.bgr_to_image_msg(frame, msg.header))
                    return
                now = time.time()
                should_log = (now - self.last_log_time) >= self.min_log_dt
                base_from_frame_tf = None
                need_base_tf_for_target_pose = (
                    self.publish_target_pose_in_base_frame
                    and self.target_marker_id >= 0
                    and bool(self.base_frame_id)
                )
                need_base_tf_for_base_tf_pub = (
                    self.publish_tf
                    and self.publish_base_frame_tf
                    and bool(self.base_frame_id)
                )
                if need_base_tf_for_target_pose or need_base_tf_for_base_tf_pub:
                    base_from_frame_tf = self.lookup_base_from_frame(frame_id, msg.header.stamp)

                for i, marker_id in enumerate(ids.flatten()):
                    pose_est = pose_estimates[i]
                    marker_id_int = int(marker_id)
                    if pose_est is None:
                        cv2.putText(
                            frame,
                            f"ID:{marker_id_int} pose solve failed",
                            (10, 30 + 24 * i),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 165, 255),
                            2,
                            cv2.LINE_AA,
                        )
                        continue

                    rvec, tvec = pose_est
                    x, y, z = tvec.flatten()
                    marker_pose = self.pose_from_rvec_tvec(rvec, tvec)
                    marker_pose = self.rotate_pose_about_local_z(
                        marker_pose, self.pose_z_rotation_rad
                    )
                    pose_array_msg.poses.append(marker_pose)
                    box_pose_in_base = None

                    if self.publish_tf and frame_id:
                        tf_msg = TransformStamped()
                        tf_msg.header.stamp = msg.header.stamp
                        tf_msg.header.frame_id = frame_id
                        tf_msg.child_frame_id = f"{self.tf_child_frame_prefix}{marker_id_int}"
                        tf_msg.transform.translation.x = marker_pose.position.x
                        tf_msg.transform.translation.y = marker_pose.position.y
                        tf_msg.transform.translation.z = marker_pose.position.z
                        tf_msg.transform.rotation.x = marker_pose.orientation.x
                        tf_msg.transform.rotation.y = marker_pose.orientation.y
                        tf_msg.transform.rotation.z = marker_pose.orientation.z
                        tf_msg.transform.rotation.w = marker_pose.orientation.w
                        tf_msgs.append(tf_msg)
                    if base_from_frame_tf is not None:
                        box_pose_in_base = self.transform_pose(base_from_frame_tf, marker_pose)
                        if self.publish_tf and self.publish_base_frame_tf:
                            tf_base_msg = TransformStamped()
                            tf_base_msg.header.stamp = msg.header.stamp
                            tf_base_msg.header.frame_id = self.base_frame_id
                            tf_base_msg.child_frame_id = (
                                f"{self.base_tf_child_frame_prefix}{marker_id_int}"
                            )
                            tf_base_msg.transform.translation.x = box_pose_in_base.position.x
                            tf_base_msg.transform.translation.y = box_pose_in_base.position.y
                            tf_base_msg.transform.translation.z = box_pose_in_base.position.z
                            tf_base_msg.transform.rotation.x = box_pose_in_base.orientation.x
                            tf_base_msg.transform.rotation.y = box_pose_in_base.orientation.y
                            tf_base_msg.transform.rotation.z = box_pose_in_base.orientation.z
                            tf_base_msg.transform.rotation.w = box_pose_in_base.orientation.w
                            tf_msgs.append(tf_base_msg)

                    if (
                        self.target_marker_id >= 0
                        and marker_id_int == self.target_marker_id
                    ):
                        target_pose_msg = PoseStamped()
                        target_pose_msg.header.stamp = msg.header.stamp
                        if (
                            self.publish_target_pose_in_base_frame
                            and box_pose_in_base is not None
                        ):
                            target_pose_msg.header.frame_id = self.base_frame_id
                            target_pose_msg.pose = box_pose_in_base
                        else:
                            target_pose_msg.header.frame_id = frame_id
                            target_pose_msg.pose = marker_pose
                            if (
                                self.publish_target_pose_in_base_frame
                                and should_log
                            ):
                                self.get_logger().warn(
                                    "Base-frame target pose requested, but TF lookup failed. "
                                    f"Publishing target pose in '{frame_id}' instead."
                                )

                    if self.draw_axes:
                        cv2.drawFrameAxes(
                            frame,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvec,
                            tvec,
                            self.marker_length * 0.6,
                        )
                    cv2.putText(
                        frame,
                        f"ID:{marker_id_int} x:{x:.3f} y:{y:.3f} z:{z:.3f} m",
                        (10, 30 + 24 * i),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

                    if should_log:
                        if box_pose_in_base is not None:
                            bx = box_pose_in_base.position.x
                            by = box_pose_in_base.position.y
                            bz = box_pose_in_base.position.z
                            self.get_logger().info(
                                f"Marker {marker_id_int:>3d} pose wrt {self.base_frame_id} [m]: "
                                f"x={bx:+.4f}, y={by:+.4f}, z={bz:+.4f}"
                            )
                        else:
                            self.get_logger().warn(
                                f"Marker {marker_id_int:>3d} base pose unavailable; "
                                f"failed to resolve transform to '{self.base_frame_id}'."
                            )

                if should_log:
                    self.last_log_time = now
            else:
                cv2.putText(
                    frame,
                    "Waiting for /camera/color/camera_info",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
        else:
            cv2.putText(
                frame,
                "No markers detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        self.poses_pub.publish(pose_array_msg)
        self.marker_ids_pub.publish(marker_ids_msg)
        if target_pose_msg is not None:
            self.target_pose_pub.publish(target_pose_msg)
        if tf_msgs:
            self.tf_broadcaster.sendTransform(tf_msgs)
        self.preview_pub.publish(self.bgr_to_image_msg(frame, msg.header))

        if self.show_window:
            try:
                cv2.imshow(self.window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    self.get_logger().info("Shutdown requested from preview window.")
                    rclpy.shutdown()
            except cv2.error as exc:
                self.get_logger().error(
                    f"OpenCV window error ({exc}). Disabling show_window."
                )
                self.show_window = False

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> int:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module missing. Install opencv-contrib-python.")

    rclpy.init()
    node = ArucoPoseDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
