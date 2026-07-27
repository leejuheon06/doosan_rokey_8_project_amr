#!/usr/bin/env python3
"""
amr_alarm_node.py

배터리 알람과 수액 타이머 알람을 navigation 코드에서 분리한 노드.

입력
- /robot1/battery_state                 sensor_msgs/BatteryState
- /robot1/events/iv_timer_done          std_msgs/Bool

출력
- /robot1/cmd_audio                     irobot_create_msgs/AudioNoteVector
"""

import math

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class AmrAlarmNode(Node):
    """배터리/수액 알람 전담 노드."""

    def __init__(self):
        super().__init__("amr_alarm_node")

        self.declare_parameter("robot_namespace", "/robot1")
        self.declare_parameter("battery_low_threshold", 0.80)
        self.declare_parameter("battery_recover_threshold", 0.81)

        self.ns = self.get_parameter("robot_namespace").value.rstrip("/")
        if not self.ns.startswith("/"):
            self.ns = "/" + self.ns

        self.battery_topic = f"{self.ns}/battery_state"
        self.iv_timer_done_topic = f"{self.ns}/events/iv_timer_done"
        self.audio_topic = f"{self.ns}/cmd_audio"

        self.battery_alarm_sent = False
        self.iv_alarm_active = False

        self.create_subscription(BatteryState, self.battery_topic, self.battery_callback, 10)
        self.create_subscription(Bool, self.iv_timer_done_topic, self.iv_timer_done_callback, 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, self.audio_topic, 10)

        self.get_logger().info("amr_alarm_node started.")
        self.get_logger().info(f"namespace={self.ns}")

    def battery_callback(self, msg: BatteryState):
        """배터리 퍼센트가 기준 이하가 되면 1회 알람."""
        percentage = float(msg.percentage)
        if math.isnan(percentage) or percentage < 0.0:
            return

        low = float(self.get_parameter("battery_low_threshold").value)
        recover = float(self.get_parameter("battery_recover_threshold").value)

        if percentage <= low and not self.battery_alarm_sent:
            self.battery_alarm_sent = True
            self.get_logger().warn(f"Battery low: {percentage * 100.0:.1f}%")
            self.play_alarm("battery")
            return

        if percentage >= recover and self.battery_alarm_sent:
            self.battery_alarm_sent = False
            self.get_logger().info(f"Battery recovered: {percentage * 100.0:.1f}%")

    def iv_timer_done_callback(self, msg: Bool):
        """수액 타이머 완료 이벤트 알람."""
        if msg.data:
            self.iv_alarm_active = True
            self.get_logger().warn("IV timer done. Play alarm.")
            self.play_alarm("iv_timer")

    def play_alarm(self, reason: str):
        """Create3 cmd_audio로 알람 패턴 publish."""
        short = DurationMsg(sec=0, nanosec=200_000_000)
        medium = DurationMsg(sec=0, nanosec=300_000_000)
        long = DurationMsg(sec=0, nanosec=500_000_000)

        patterns = {
            "battery": [
                (330, medium),
                (262, medium),
                (330, medium),
                (262, long),
            ],
            "iv_timer": [
                (1200, short),
                (900, short),
                (1200, short),
                (900, medium),
            ],
        }

        msg = AudioNoteVector()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.append = False
        msg.notes = [AudioNote(frequency=freq, max_runtime=duration)
                     for freq, duration in patterns.get(reason, patterns["iv_timer"])]
        self.audio_pub.publish(msg)
        self.get_logger().info(f"Alarm published: {reason}")


def main():
    rclpy.init()
    node = AmrAlarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
