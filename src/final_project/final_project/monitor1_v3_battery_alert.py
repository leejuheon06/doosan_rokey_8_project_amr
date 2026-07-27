#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from flask import Flask, render_template, jsonify

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.action import Dock
from irobot_create_msgs.msg import DockStatus
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Int32


app = Flask(__name__)


# ===== ROS2 토픽 / 액션 이름 설정 =====
UNDOCK_SIGNAL_TOPIC = '/robot1/undock_signal'
DOCK_ACTION_NAME = '/robot1/dock'
DOCK_STATUS_TOPIC = '/robot1/dock_status'
BATTERY_STATE_TOPIC = '/robot1/battery_state'
NURSE_CALL_TOPIC = '/monitor1/nurse_call'

# TurtleBot4 HMI LED hidden status topics
# turtlebot4_base_node가 실제 HMI LED 상태를 읽는 내부 토픽입니다.
LED_POWER_TOPIC = '/robot1/hmi/led/_power'
LED_MOTORS_TOPIC = '/robot1/hmi/led/_motors'
LED_COMMS_TOPIC = '/robot1/hmi/led/_comms'
LED_WIFI_TOPIC = '/robot1/hmi/led/_wifi'
LED_BATTERY_TOPIC = '/robot1/hmi/led/_battery'
# ===================================


# 최근 메시지 기준으로 연결 끊김을 판단할 시간
CONNECTION_TIMEOUT_SEC = 3.0
LED_TIMEOUT_SEC = 3.0

# 배터리 경고 기준값입니다. BatteryState.percentage는 0.0~1.0이므로 화면 표시값 기준 50.0%로 판단합니다.
LOW_BATTERY_WARNING_THRESHOLD = 80.0


# Monitor1 화면에 표시할 상태값 저장용 전역 딕셔너리
amr1_status = {
    "battery": "--",
    "battery_voltage": "--",
    "battery_current": "--",
    "battery_temperature": "--",
    "state": "Not connect",
    "connected": False,

    # HTML에서 50% 이하 배터리 경고 팝업을 1회 띄우기 위한 값입니다.
    # get_status_snapshot()에서 True를 한 번 반환한 뒤 False로 초기화합니다.
    "battery_warning_popup": False,
    "battery_warning_message": "",

    "leds": {
        "power": "off",
        "motors": "off",
        "comms": "off",
        "wifi": "off",
        "battery": "off",
    }
}


ros_node = None


class Monitor1RosNode(Node):

    def __init__(self):
        super().__init__('monitor1_node')

        self.lock = threading.Lock()
        # 여러 subscriber/action/Flask 요청이 동시에 들어와도 ROS callback 처리가 막히지 않도록
        # ReentrantCallbackGroup + MultiThreadedExecutor 조합을 사용합니다.
        self.callback_group = ReentrantCallbackGroup()
        now = time.time()

        # 최근 수신 시간 저장. 일정 시간 이상 수신이 없으면 연결 끊김으로 표시합니다.
        self.last_battery_time = 0.0
        self.last_dock_status_time = 0.0
        self.last_led_time = {
            "power": 0.0,
            "motors": 0.0,
            "comms": 0.0,
            "wifi": 0.0,
            "battery": 0.0,
        }

        # LED 원본 값 저장. std_msgs/msg/Int32: 0=OFF, 1=GREEN, 2=RED, 3=YELLOW 로 처리합니다.
        self.led_values = {
            "power": None,
            "motors": None,
            "comms": None,
            "wifi": None,
            "battery": None,
        }

        # 배터리 경고 팝업 중복 방지 플래그입니다.
        # True가 되면 배터리가 다시 50%보다 높아질 때까지 같은 경고를 다시 만들지 않습니다.
        self.low_battery_warning_sent = False

        # Flask /api/status가 HTML에 전달할 1회성 팝업 대기 플래그입니다.
        # battery_callback()에서 True로 만들고, get_status_snapshot()에서 한 번 전달한 뒤 False로 되돌립니다.
        self.low_battery_popup_pending = False

        self.undock_signal = self.create_publisher(
            Bool,
            UNDOCK_SIGNAL_TOPIC,
            10
        )

        self.nurse_call_pub = self.create_publisher(
            Bool,
            NURSE_CALL_TOPIC,
            10
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            BATTERY_STATE_TOPIC,
            self.battery_callback,
            10,
            callback_group=self.callback_group
        )

        self.dock_status_sub = self.create_subscription(
            DockStatus,
            DOCK_STATUS_TOPIC,
            self.dock_status_callback,
            10,
            callback_group=self.callback_group
        )

        # TurtleBot4 HMI LED 상태 토픽 구독
        self.led_power_sub = self.create_subscription(
            Int32,
            LED_POWER_TOPIC,
            lambda msg: self.led_callback("power", msg),
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )
        self.led_motors_sub = self.create_subscription(
            Int32,
            LED_MOTORS_TOPIC,
            lambda msg: self.led_callback("motors", msg),
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )
        self.led_comms_sub = self.create_subscription(
            Int32,
            LED_COMMS_TOPIC,
            lambda msg: self.led_callback("comms", msg),
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )
        self.led_wifi_sub = self.create_subscription(
            Int32,
            LED_WIFI_TOPIC,
            lambda msg: self.led_callback("wifi", msg),
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )
        self.led_battery_sub = self.create_subscription(
            Int32,
            LED_BATTERY_TOPIC,
            lambda msg: self.led_callback("battery", msg),
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        self.dock_action_client = ActionClient(
            self,
            Dock,
            DOCK_ACTION_NAME,
            callback_group=self.callback_group
        )

        self.get_logger().info("Monitor1 ROS2 node started")
        self.get_logger().info("TurtleBot4 LED status subscribers started")

    def battery_callback(self, msg):
        # /robot1/battery_state 토픽이 들어올 때마다 실행되는 콜백입니다.
        with self.lock:
            # 마지막 배터리 메시지 수신 시간을 저장해 Connected/Disconnected 판단에 사용합니다.
            self.last_battery_time = time.time()

            # BatteryState.percentage는 0.0~1.0 범위이므로 화면 표시용으로 0~100%로 변환합니다.
            if msg.percentage >= 0.0:
                battery_percent = round(msg.percentage * 100, 1)
                amr1_status["battery"] = battery_percent

                # 배터리가 50% 이하로 처음 떨어진 순간에만 팝업을 예약합니다.
                # 예: 50.0%, 49.8% 등 50% 이하가 처음 감지되면 한 번만 동작합니다.
                if battery_percent <= LOW_BATTERY_WARNING_THRESHOLD and not self.low_battery_warning_sent:
                    self.low_battery_warning_sent = True
                    self.low_battery_popup_pending = True
                    amr1_status["battery_warning_message"] = (
                        f"⚠ Battery level is {battery_percent:.1f}%. Please prepare to dock."
                    )
                    self.get_logger().warn(amr1_status["battery_warning_message"])

                # 충전 등으로 배터리가 다시 50%보다 높아지면 다음 하강 때 다시 한 번 경고할 수 있게 재무장합니다.
                if battery_percent > LOW_BATTERY_WARNING_THRESHOLD:
                    self.low_battery_warning_sent = False
            else:
                # percentage 값이 유효하지 않으면 화면에는 미수신 상태로 표시합니다.
                amr1_status["battery"] = "--"

            # BatteryState의 voltage/current/temperature 값도 HTML에 표시합니다.
            amr1_status["battery_voltage"] = round(msg.voltage, 2) if msg.voltage >= 0.0 else "--"
            amr1_status["battery_current"] = round(msg.current, 2)
            amr1_status["battery_temperature"] = round(msg.temperature, 1) if msg.temperature >= 0.0 else "--"

    def dock_status_callback(self, msg):
        with self.lock:
            self.last_dock_status_time = time.time()
            amr1_status["state"] = "DOCK" if msg.is_docked else "UNDOCK"

    def led_callback(self, led_name, msg):
        with self.lock:
            self.led_values[led_name] = int(msg.data)
            self.last_led_time[led_name] = time.time()

    def led_value_to_state(self, value):
        # turtlebot4_base_node의 hmi/led/_[led]는 std_msgs/msg/Int32 hidden topic입니다.
        # 일반적으로 UserLed color enum과 같이 0=OFF, 1=GREEN, 2=RED, 3=YELLOW 로 처리합니다.
        if value is None:
            return "off"
        if value == 0:
            return "off"
        if value == 1:
            return "green"
        if value == 2:
            return "red"
        if value == 3:
            return "yellow"
        return "unknown"

    def get_status_snapshot(self):
        now = time.time()
        with self.lock:
            # /robot1/battery_state가 최근 3초 이내에 들어왔는지 확인합니다.
            battery_recent = (
                now - self.last_battery_time <= CONNECTION_TIMEOUT_SEC
            )

            # /robot1/dock_status가 최근 3초 이내에 들어왔는지 확인합니다.
            dock_recent = (
                now - self.last_dock_status_time <= CONNECTION_TIMEOUT_SEC
            )

            # BatteryState와 DockStatus가 둘 다 최근 3초 이내에 들어온 경우에만
            # 오른쪽 상단 연결 상태를 Connected로 판단합니다.
            recent_core_msg = battery_recent and dock_recent

            leds = {}
            for name, value in self.led_values.items():
                if now - self.last_led_time[name] <= LED_TIMEOUT_SEC:
                    leds[name] = self.led_value_to_state(value)
                else:
                    leds[name] = "off"

            # 실제 LED hidden topic이 아직 안 들어오는 환경을 위한 최소 fallback입니다.
            # 연결이 살아있는데 LED topic만 안 들어오면 power/comms/wifi는 연결 상태 기준으로 표시합니다.
            if recent_core_msg and all(v == "off" for v in leds.values()):
                leds["power"] = "green"
                leds["comms"] = "green"
                leds["wifi"] = "green"
                if isinstance(amr1_status["battery"], (int, float)):
                    if amr1_status["battery"] >= 50.0:
                        leds["battery"] = "green"
                    elif amr1_status["battery"] >= 20.0:
                        leds["battery"] = "yellow"
                    else:
                        leds["battery"] = "red"
                else:
                    leds["battery"] = "off"

            amr1_status["connected"] = bool(recent_core_msg)
            amr1_status["leds"] = leds

            # 배터리 경고 팝업은 1회성 이벤트입니다.
            # True를 HTML로 한 번 보낸 뒤 바로 False로 초기화해 새로고침마다 반복 팝업이 뜨지 않게 합니다.
            battery_warning_popup = self.low_battery_popup_pending
            battery_warning_message = amr1_status["battery_warning_message"]
            self.low_battery_popup_pending = False
            amr1_status["battery_warning_popup"] = False

            # dict 복사본을 반환해서 Flask thread와 ROS thread 간 데이터 충돌을 줄입니다.
            return {
                "battery": amr1_status["battery"],
                "battery_voltage": amr1_status["battery_voltage"],
                "battery_current": amr1_status["battery_current"],
                "battery_temperature": amr1_status["battery_temperature"],
                "state": amr1_status["state"],
                "connected": amr1_status["connected"],
                "leds": dict(amr1_status["leds"]),
                "battery_warning_popup": battery_warning_popup,
                "battery_warning_message": battery_warning_message,
            }

    def publish_undock_siganl(self):
        msg = Bool()
        msg.data = True
        self.undock_signal.publish(msg)
        self.get_logger().info("Undock signal published: True")

    def send_dock_action(self):
        if not self.dock_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"Dock action server is not available: {DOCK_ACTION_NAME}")
            return False, "Dock action server is not available"

        goal_msg = Dock.Goal()
        self.dock_action_client.send_goal_async(goal_msg)
        self.get_logger().info("Dock action goal sent")
        return True, "Dock action signal sent"

    def publish_nurse_call(self):
        msg = Bool()
        msg.data = True
        self.nurse_call_pub.publish(msg)
        self.get_logger().info("Nurse call signal published")


def run_ros2_node():
    global ros_node
    rclpy.init()
    ros_node = Monitor1RosNode()

    # Flask thread에서 /api/dock, /api/undock 요청이 들어오는 동안에도
    # 배터리, dock_status, LED subscriber callback이 밀리지 않도록 MultiThreadedExecutor 사용.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ros_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


@app.route('/')
def index():
    return render_template('monitor1_led_battery_alert.html')


@app.route('/api/status', methods=['GET'])
def get_status():
    if ros_node:
        return jsonify(ros_node.get_status_snapshot())
    return jsonify(amr1_status)


@app.route('/api/dock', methods=['POST'])
def dock():
    if amr1_status["state"] == "DOCK":
        return jsonify({
            "success": False,
            "message": "이미 Dock 상태입니다."
        })

    if amr1_status["state"] == "Not connect":
        return jsonify({
            "success": False,
            "message": "현재 IV Pole 상태를 수신하지 못했습니다."
        })

    if not ros_node:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        })

    success, message = ros_node.send_dock_action()
    return jsonify({
        "success": success,
        "message": message
    })


@app.route('/api/undock', methods=['POST'])
def undock():
    if amr1_status["state"] == "UNDOCK":
        return jsonify({
            "success": False,
            "message": "이미 Undock 상태입니다."
        })

    if amr1_status["state"] == "Not connect":
        return jsonify({
            "success": False,
            "message": "현재 IV Pole 상태를 수신하지 못했습니다."
        })

    if not ros_node:
        return jsonify({
            "success": False,
            "message": "ROS2 node is not ready"
        })

    ros_node.publish_undock_siganl()
    return jsonify({
        "success": True,
        "message": "Undock command signal sent"
    })


@app.route('/api/nurse-call', methods=['POST'])
def nurse_call():
    if ros_node:
        ros_node.publish_nurse_call()

    return jsonify({
        "success": True,
        "message": "간호사 호출 신호를 전송했습니다."
    })


# 이 파일을 직접 실행했을 때만 아래 코드 실행
def main():
    # ROS2 노드를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=run_ros2_node, daemon=True)

    # ROS2 스레드 시작
    ros_thread.start()

    # Flask 웹 서버 실행
    app.run(debug=False, host='0.0.0.0', port=5000)


# 이 파일을 직접 스크립트로 실행했을 때도 작동하도록 처리
if __name__ == '__main__':
    main()
