#!/usr/bin/env python3
"""
smooth_follow_navigation.py

TurtleBot4 사람/목표 객체 추종 전용 Navigation 노드.

핵심 설계
- YOLO 추론은 하지 않고, 외부 yolo_tracker.py가 publish하는 tracked_target 토픽만 사용한다.
- 사람을 따라가는 동안에는 Nav2 goal을 반복 전송하지 않고 cmd_vel로 직접 제어한다.
  → 움직이는 사람, 좁은 카메라 화각 상황에서 반응이 빠르고 흔들림이 적다.
- monitor 코드가 /robot1/dock action을 직접 호출하는 구조를 고려하여,
  /robot1/dock_status의 False -> True 전이를 감지하면 추종을 즉시 정리한다.
- 배터리/수액 알람은 별도 amr_alarm_node.py로 분리했다.

입력 토픽 예시
- /robot1/events/start_patient_follow        std_msgs/Bool
- /robot1/undock_signal                      std_msgs/Bool
- /robot1/dock_status                        irobot_create_msgs/DockStatus
- /robot1/tracked_target/detected            std_msgs/Bool
- /robot1/tracked_target/label               std_msgs/String
- /robot1/tracked_target/confidence          std_msgs/Float32
- /robot1/tracked_target/center_x            std_msgs/Float32
- /robot1/tracked_target/center_y            std_msgs/Float32
- /robot1/tracked_target/depth_m             std_msgs/Float32

출력 토픽
- /robot1/cmd_vel                            geometry_msgs/Twist
- /robot1/events/yolo_tracker_enable         std_msgs/Bool
"""

import math
import threading
import time
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String


class SmoothFollowNavigation(Node):
    """TurtleBot4 목표 객체 추종 노드."""

    IDLE_DOCKED = "IDLE_DOCKED"
    UNDOCKING = "UNDOCKING"
    FOLLOWING_TARGET = "FOLLOWING_TARGET"
    IDLE_UNDOCKED = "IDLE_UNDOCKED"

    def __init__(self):
        super().__init__("smooth_follow_navigation")

        self.lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()

        # -------------------------
        # ROS parameters
        # -------------------------
        # robot_namespace는 '/robot1' 형식으로 사용한다.
        self.declare_parameter("robot_namespace", "/robot1")
        self.declare_parameter("target_label", "orange")
        self.declare_parameter("min_confidence", 0.70)
        self.declare_parameter("tracker_image_width", 704.0)

        # 유지하고 싶은 거리. 실제 추종은 이 거리 근처에 머물도록 한다.
        self.declare_parameter("target_distance_m", 0.8)
        self.declare_parameter("distance_deadband_m", 0.12)
        self.declare_parameter("valid_depth_min_m", 0.25)
        self.declare_parameter("valid_depth_max_m", 4.50)

        # 속도 제한. TurtleBot4 실내 사람 추종에서는 처음에는 작게 두는 것이 안전하다.
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 0.50)
        self.declare_parameter("max_linear_accel", 0.35)    # m/s^2
        self.declare_parameter("max_angular_accel", 1.20)   # rad/s^2

        # 제어 gain. 사람이 화면 밖으로 자주 나가면 angular_kp를 조금 올린다.
        self.declare_parameter("linear_kp", 0.45)
        self.declare_parameter("angular_kp", 0.0032)

        # 영상 중심 근처에서는 회전을 멈춰 미세한 bbox 흔들림에 반응하지 않게 한다.
        self.declare_parameter("pixel_deadband", 30.0)

        # EMA 필터 계수. 작을수록 부드럽지만 반응이 느리다.
        self.declare_parameter("depth_filter_alpha", 0.35)
        self.declare_parameter("pixel_filter_alpha", 0.45)

        # target이 잠깐 끊겨도 바로 lost 처리하지 않는 시간.
        self.declare_parameter("target_stale_timeout_s", 0.35)
        self.declare_parameter("target_lost_stop_timeout_s", 0.70)

        # 언도킹 직후 도킹 스테이션에서 벗어나기 위한 짧은 전진.
        self.declare_parameter("move_forward_after_undock", True)
        self.declare_parameter("undock_forward_distance_m", 0.30)
        self.declare_parameter("undock_forward_speed_mps", 0.08)

        self.ns = self.get_parameter("robot_namespace").value.rstrip("/")
        if not self.ns.startswith("/"):
            self.ns = "/" + self.ns

        self.target_label = self.get_parameter("target_label").value.lower()
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.image_width = float(self.get_parameter("tracker_image_width").value)
        self.image_center_x = self.image_width / 2.0

        # -------------------------
        # Runtime state
        # -------------------------
        self.state = self.IDLE_DOCKED
        self.shutdown_requested = False
        self.tracking_enabled = False

        self.latest_target: Optional[Dict[str, float]] = None
        self.last_target_seen_time = 0.0
        self.target_seen_once = False

        self.filtered_depth: Optional[float] = None
        self.filtered_pixel_error: Optional[float] = None
        self.last_cmd_time = time.time()
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0

        self.dock_status: Optional[DockStatus] = None
        self.last_known_is_docked: Optional[bool] = None
        self.undock_in_progress = False

        # -------------------------
        # Topics / actions
        # -------------------------
        self.cmd_vel_topic = f"{self.ns}/cmd_vel"
        self.dock_status_topic = f"{self.ns}/dock_status"
        self.undock_action_name = f"{self.ns}/undock"

        self.start_follow_topic = f"{self.ns}/events/start_patient_follow"
        self.undock_signal_topic = f"{self.ns}/undock_signal"
        self.yolo_enable_topic = f"{self.ns}/events/yolo_tracker_enable"

        target_base = f"{self.ns}/tracked_target"
        self.detected_topic = f"{target_base}/detected"
        self.label_topic = f"{target_base}/label"
        self.confidence_topic = f"{target_base}/confidence"
        self.center_x_topic = f"{target_base}/center_x"
        self.center_y_topic = f"{target_base}/center_y"
        self.depth_topic = f"{target_base}/depth_m"

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos_profile_sensor_data)
        self.yolo_enable_pub = self.create_publisher(Bool, self.yolo_enable_topic, 10)

        self.undock_client = ActionClient(self, Undock, self.undock_action_name)

        # 외부 이벤트
        self.create_subscription(Bool, self.start_follow_topic, self.start_follow_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(Bool, self.undock_signal_topic, self.undock_signal_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(DockStatus, self.dock_status_topic, self.dock_status_callback, 10,
                                 callback_group=self.callback_group)

        # YOLO tracker 분리 토픽
        self.create_subscription(Bool, self.detected_topic, self.detected_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(String, self.label_topic, self.label_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(Float32, self.confidence_topic, self.confidence_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(Float32, self.center_x_topic, self.center_x_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(Float32, self.center_y_topic, self.center_y_callback, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(Float32, self.depth_topic, self.depth_callback, 10,
                                 callback_group=self.callback_group)

        # 20Hz 제어 루프. Nav2 goal 방식보다 빠르게 반응한다.
        self.control_timer = self.create_timer(0.05, self.control_loop,
                                               callback_group=self.callback_group)

        self.get_logger().info("smooth_follow_navigation node started.")
        self.get_logger().info(f"namespace={self.ns}, target_label={self.target_label}")
        self.get_logger().info("Follow mode: direct cmd_vel distance + heading control")

    # ------------------------------------------------------------------
    # External command callbacks
    # ------------------------------------------------------------------
    def start_follow_callback(self, msg: Bool):
        """환자 추종 시작 이벤트."""
        if msg.data:
            self.start_mission_async("start_patient_follow")

    def undock_signal_callback(self, msg: Bool):
        """외부 언도킹 신호."""
        if msg.data:
            self.start_mission_async("undock_signal")

    def start_mission_async(self, source: str):
        """action 대기 때문에 callback을 막지 않도록 별도 thread에서 미션 시작."""
        self.get_logger().info(f"Follow mission requested from {source}.")
        threading.Thread(target=self.start_follow_mission, daemon=True).start()

    def start_follow_mission(self):
        """언도킹 후 YOLO tracker를 켜고 cmd_vel 추종을 시작한다."""
        with self.lock:
            if self.state in (self.UNDOCKING, self.FOLLOWING_TARGET):
                self.get_logger().info(f"Mission ignored. current state={self.state}")
                return
            if self.dock_status is not None and not self.dock_status.is_docked:
                # 이미 도킹 해제 상태라면 언도킹 action 없이 바로 추종을 시작한다.
                self.get_logger().info("Robot already undocked. Start tracking directly.")
                self.reset_tracking_state_locked()
                self.state = self.FOLLOWING_TARGET
                self.tracking_enabled = True
                self.publish_yolo_enable(True)
                return
            self.state = self.UNDOCKING
            self.undock_in_progress = True
            self.reset_tracking_state_locked()

        self.publish_yolo_enable(False)
        undock_ok = self.undock_with_timeout(timeout_s=15.0)
        if not undock_ok:
            self.get_logger().warn("Undock failed or timed out. Tracking will not start.")
            with self.lock:
                self.state = self.IDLE_DOCKED
                self.undock_in_progress = False
            return

        if self.get_parameter("move_forward_after_undock").value:
            distance = float(self.get_parameter("undock_forward_distance_m").value)
            speed = float(self.get_parameter("undock_forward_speed_mps").value)
            self.move_forward(distance_m=distance, speed_mps=speed)

        with self.lock:
            self.state = self.FOLLOWING_TARGET
            self.tracking_enabled = True
            self.undock_in_progress = False

        self.publish_yolo_enable(True)
        self.get_logger().info("Tracking enabled. Waiting for target.")

    def dock_status_callback(self, msg: DockStatus):
        """monitor가 직접 dock action을 호출했을 때도 실제 도킹 완료를 감지한다."""
        with self.lock:
            prev = self.last_known_is_docked
            self.dock_status = msg
            self.last_known_is_docked = msg.is_docked

        if prev is None:
            self.get_logger().info(f"Dock status received. is_docked={msg.is_docked}")
            return

        if not prev and msg.is_docked:
            # 핵심: navigation을 거치지 않고 monitor가 직접 dock시켜도 여기서 추종을 정리한다.
            self.get_logger().info("Docking detected from dock_status. Cleanup follow state.")
            self.cleanup_after_docking_detected()
        elif prev and not msg.is_docked:
            self.get_logger().info("Robot undocked.")

    def cleanup_after_docking_detected(self):
        """실제 도킹 완료 시 추종 관련 상태를 정리한다."""
        self.publish_yolo_enable(False)
        self.stop_robot(repeat=8)
        with self.lock:
            self.tracking_enabled = False
            self.state = self.IDLE_DOCKED
            self.undock_in_progress = False
            self.reset_tracking_state_locked()

    # ------------------------------------------------------------------
    # YOLO tracker callbacks
    # ------------------------------------------------------------------
    def detected_callback(self, msg: Bool):
        """검출 cycle 시작/실패 여부를 저장한다."""
        now = time.time()
        with self.lock:
            if not msg.data:
                # 검출 실패 fallback 좌표가 뒤늦게 섞이지 않도록 detected만 남긴다.
                self.latest_target = {"detected": False, "received_at": now}
                return
            self.latest_target = {"detected": True, "received_at": now}

    def _accepting_target_fields_locked(self) -> bool:
        return self.latest_target is not None and bool(self.latest_target.get("detected", False))

    def label_callback(self, msg: String):
        with self.lock:
            if self._accepting_target_fields_locked():
                self.latest_target["label"] = msg.data.lower()
                self.latest_target["received_at"] = time.time()

    def confidence_callback(self, msg: Float32):
        with self.lock:
            if self._accepting_target_fields_locked():
                self.latest_target["confidence"] = float(msg.data)
                self.latest_target["received_at"] = time.time()

    def center_x_callback(self, msg: Float32):
        with self.lock:
            if self._accepting_target_fields_locked():
                # detected=False fallback으로 흔히 들어오는 0.0은 무시한다.
                if float(msg.data) <= 1.0:
                    return
                self.latest_target["center_x"] = float(msg.data)
                self.latest_target["received_at"] = time.time()

    def center_y_callback(self, msg: Float32):
        with self.lock:
            if self._accepting_target_fields_locked():
                self.latest_target["center_y"] = float(msg.data)
                self.latest_target["received_at"] = time.time()

    def depth_callback(self, msg: Float32):
        with self.lock:
            if self._accepting_target_fields_locked():
                self.latest_target["depth_m"] = float(msg.data)
                self.latest_target["received_at"] = time.time()

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        """20Hz로 target 정보를 확인하고 cmd_vel을 계산한다."""
        with self.lock:
            enabled = self.tracking_enabled
            state = self.state
            target = dict(self.latest_target) if self.latest_target is not None else None

        if not enabled or state != self.FOLLOWING_TARGET:
            return

        now = time.time()
        valid_target = self.validate_target(target, now)
        if valid_target is None:
            self.handle_target_missing(now)
            return

        label = valid_target["label"]
        depth = valid_target["depth_m"]
        center_x = valid_target["center_x"]

        if not self.target_seen_once:
            self.get_logger().info(f"First target detected: {label}, depth={depth:.2f}m")
        self.target_seen_once = True
        self.last_target_seen_time = now

        # 필터링: bbox/depth의 순간 튐을 줄인다.
        pixel_error_raw = center_x - self.image_center_x
        pixel_error = self.low_pass_pixel(pixel_error_raw)
        depth_filtered = self.low_pass_depth(depth)

        linear_x, angular_z = self.compute_follow_cmd(depth_filtered, pixel_error)
        linear_x, angular_z = self.limit_acceleration(linear_x, angular_z, now)

        self.publish_cmd(linear_x, angular_z)

    def validate_target(self, target: Optional[Dict[str, float]], now: float) -> Optional[Dict[str, float]]:
        """추종에 사용할 수 있는 target인지 검사한다."""
        if target is None:
            return None
        if not target.get("detected", False):
            return None
        if now - float(target.get("received_at", 0.0)) > float(self.get_parameter("target_stale_timeout_s").value):
            return None

        required = ("label", "confidence", "center_x", "depth_m")
        if any(key not in target for key in required):
            return None

        label = str(target["label"]).lower()
        confidence = float(target["confidence"])
        center_x = float(target["center_x"])
        depth = float(target["depth_m"])

        if label != self.target_label:
            return None
        if confidence < self.min_confidence:
            return None
        if not (0.0 < center_x < self.image_width):
            return None
        if not (float(self.get_parameter("valid_depth_min_m").value) < depth < float(self.get_parameter("valid_depth_max_m").value)):
            return None

        return target

    def handle_target_missing(self, now: float):
        """target을 놓쳤을 때는 방향을 바꾸지 않고 정지한다."""
        if not self.target_seen_once:
            # 아직 한 번도 못 봤으면 회전 탐색하지 않고 대기한다.
            self.publish_zero_velocity()
            return

        lost_elapsed = now - self.last_target_seen_time
        if lost_elapsed >= float(self.get_parameter("target_lost_stop_timeout_s").value):
            # 좁은 화각에서 임의 회전하면 더 쉽게 놓치므로, lost 시에는 정지 유지가 기본이다.
            self.publish_zero_velocity()
            self.filtered_depth = None
            self.filtered_pixel_error = None
            self.last_linear_cmd = 0.0
            self.last_angular_cmd = 0.0
        else:
            # 아주 짧은 순간 끊김은 이전 속도를 끌고 가지 않고 바로 감속 정지한다.
            linear_x, angular_z = self.limit_acceleration(0.0, 0.0, now)
            self.publish_cmd(linear_x, angular_z)

    def low_pass_depth(self, depth: float) -> float:
        alpha = float(self.get_parameter("depth_filter_alpha").value)
        if self.filtered_depth is None:
            self.filtered_depth = depth
        else:
            self.filtered_depth = alpha * depth + (1.0 - alpha) * self.filtered_depth
        return self.filtered_depth

    def low_pass_pixel(self, pixel_error: float) -> float:
        alpha = float(self.get_parameter("pixel_filter_alpha").value)
        if self.filtered_pixel_error is None:
            self.filtered_pixel_error = pixel_error
        else:
            self.filtered_pixel_error = alpha * pixel_error + (1.0 - alpha) * self.filtered_pixel_error
        return self.filtered_pixel_error

    def compute_follow_cmd(self, depth_m: float, pixel_error: float):
        """거리 오차와 화면 중심 오차를 cmd_vel로 변환한다."""
        target_distance = float(self.get_parameter("target_distance_m").value)
        distance_deadband = float(self.get_parameter("distance_deadband_m").value)
        linear_kp = float(self.get_parameter("linear_kp").value)
        angular_kp = float(self.get_parameter("angular_kp").value)
        pixel_deadband = float(self.get_parameter("pixel_deadband").value)

        max_linear = float(self.get_parameter("max_linear_speed").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)

        # 거리 제어: 목표 거리보다 멀면 전진, 가까우면 정지한다.
        # 사람 추종에서는 후진이 오히려 위험할 수 있어 기본적으로 후진하지 않는다.
        distance_error = depth_m - target_distance
        if abs(distance_error) <= distance_deadband:
            linear_x = 0.0
        elif distance_error > 0.0:
            linear_x = linear_kp * distance_error
        else:
            linear_x = 0.0
        linear_x = self.clamp(linear_x, 0.0, max_linear)

        # 헤딩 제어: target이 왼쪽이면 +angular.z, 오른쪽이면 -angular.z.
        if abs(pixel_error) <= pixel_deadband:
            angular_z = 0.0
        else:
            angular_z = -angular_kp * pixel_error
        angular_z = self.clamp(angular_z, -max_angular, max_angular)

        # target이 화면 가장자리로 가면 전진보다 회전을 우선한다.
        # 좁은 화각에서 target을 잃는 것을 줄이는 핵심 안전장치다.
        edge_ratio = abs(pixel_error) / self.image_center_x
        if edge_ratio > 0.65:
            linear_x *= 0.35
        elif edge_ratio > 0.45:
            linear_x *= 0.60

        return linear_x, angular_z

    def limit_acceleration(self, desired_linear: float, desired_angular: float, now: float):
        """속도 명령 변화량을 제한해 급출발/급회전을 줄인다."""
        dt = max(0.001, now - self.last_cmd_time)
        self.last_cmd_time = now

        max_lin_step = float(self.get_parameter("max_linear_accel").value) * dt
        max_ang_step = float(self.get_parameter("max_angular_accel").value) * dt

        linear = self.last_linear_cmd + self.clamp(desired_linear - self.last_linear_cmd,
                                                   -max_lin_step, max_lin_step)
        angular = self.last_angular_cmd + self.clamp(desired_angular - self.last_angular_cmd,
                                                     -max_ang_step, max_ang_step)

        self.last_linear_cmd = linear
        self.last_angular_cmd = angular
        return linear, angular

    # ------------------------------------------------------------------
    # Robot actions / utilities
    # ------------------------------------------------------------------
    def undock_with_timeout(self, timeout_s: float) -> bool:
        """TurtleBot4 /undock action 실행."""
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"Undock action server unavailable: {self.undock_action_name}")
            return False

        future = self.undock_client.send_goal_async(Undock.Goal())
        start = time.time()
        while rclpy.ok() and not future.done() and time.time() - start < timeout_s:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().warn("Undock goal request timed out.")
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Undock goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done() and time.time() - start < timeout_s:
            time.sleep(0.05)

        if not result_future.done():
            self.get_logger().warn("Undock result timed out.")
            return False

        self.get_logger().info("Undock finished.")
        return True

    def move_forward(self, distance_m: float, speed_mps: float):
        """언도킹 직후 짧게 전진."""
        if speed_mps <= 0.0:
            return
        duration_s = distance_m / speed_mps
        twist = Twist()
        twist.linear.x = speed_mps
        start = time.time()
        while rclpy.ok() and time.time() - start < duration_s:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.stop_robot(repeat=5)

    def publish_yolo_enable(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.yolo_enable_pub.publish(msg)
        self.get_logger().info(f"YOLO tracker enable={msg.data}")

    def publish_cmd(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def stop_robot(self, repeat: int = 5):
        stop = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.02)
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        self.last_cmd_time = time.time()

    def reset_tracking_state_locked(self):
        """lock을 잡은 상태에서 호출하는 추종 상태 초기화."""
        self.tracking_enabled = False
        self.latest_target = None
        self.last_target_seen_time = 0.0
        self.target_seen_once = False
        self.filtered_depth = None
        self.filtered_pixel_error = None
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        self.last_cmd_time = time.time()

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main():
    rclpy.init()
    node = SmoothFollowNavigation()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_yolo_enable(False)
        node.stop_robot(repeat=10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
