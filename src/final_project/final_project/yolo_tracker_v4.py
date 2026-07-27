#!/usr/bin/env python3
"""
OAK-D RGB(compressed) + Stereo(depth, 16UC1/mm) 토픽 구독 + YOLO 검출 노드.

- 'robot1/oakd/rgb/image_raw/compressed' : RGB 압축 이미지 -> YOLO 추론 후 cv2.imshow로 표시
- 'robot1/oakd/stereo/image_raw'         : depth 이미지(16UC1, mm 단위) -> 화면엔 안 띄우고 거리값 조회용으로만 사용

검출된 객체(가장 confidence 높은 1개) 중심 좌표와 그 위치의 거리(m)를
좌측 상단에 표시합니다. 검출된 객체가 없으면 화면 중앙 좌표/거리를 표시합니다.
검출 confidence는 우측 상단에 별도로 표시합니다.

추가로, navigation node가 사용할 수 있도록 검출 결과를 필드별 독립 토픽(6개,
std_msgs 표준 타입)으로 발행합니다:
  /robot1/tracked_target/detected   (std_msgs/Bool)
  /robot1/tracked_target/label      (std_msgs/String)
  /robot1/tracked_target/confidence (std_msgs/Float32)
  /robot1/tracked_target/center_x   (std_msgs/Float32)
  /robot1/tracked_target/center_y   (std_msgs/Float32)
  /robot1/tracked_target/depth_m    (std_msgs/Float32)
검출된 객체가 없는 경우에도 모든 필드에 기본값(False/""/0.0)을 발행합니다.

노드 시작 시에는 '/robot1/events/yolo_tracker_enable' (std_msgs/Bool) 토픽만 구독한 채
대기하며, 카메라 구독/추론/화면 표시/발행은 전혀 동작하지 않습니다.

이 토픽은 1회성 트리거가 아니라 "활성화/비활성화 토글" 신호로 동작합니다:
  - True  수신 -> 비활성 상태일 때만 카메라 구독/타이머/창/발행자를 생성하고 동작을 시작합니다.
  - False 수신 -> 활성 상태일 때만 구독자/타이머/창/발행자를 정리하고 다시 대기 상태로 돌아갑니다.
노드 프로세스는 종료되지 않고 계속 살아 있으며, 이후 True가 다시 들어오면
몇 번이든 다시 동작을 시작할 수 있습니다 (예: navigation node가 도킹 시 False를
보내고, 다음 미션에서 언도킹 후 다시 True를 보내는 경우).
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String, Float32
from cv_bridge import CvBridge
from ultralytics import YOLO

# ---------------------------------------------------------------------
# YOLO 모델(.pt) 절대경로 - 실제 환경에 맞게 이 줄만 수정하세요.
# ---------------------------------------------------------------------
YOLO_MODEL_PATH = '/home/rokey/rokey_pjt/src/final_project/best.pt'

# ---------------------------------------------------------------------
# 이 토픽으로 활성화(True)/비활성화(False) 신호를 받는다 (반복 토글 가능).
# ---------------------------------------------------------------------
YOLO_TRACKER_ENABLE_TOPIC = '/robot1/events/yolo_tracker_enable'

# ---------------------------------------------------------------------
# navigation node로 발행하는 토픽 (6개로 분리, std_msgs 표준 타입)
# ---------------------------------------------------------------------
TOPIC_DETECTED = '/robot1/tracked_target/detected'      # std_msgs/Bool
TOPIC_LABEL = '/robot1/tracked_target/label'            # std_msgs/String
TOPIC_CONFIDENCE = '/robot1/tracked_target/confidence'  # std_msgs/Float32
TOPIC_CENTER_X = '/robot1/tracked_target/center_x'      # std_msgs/Float32
TOPIC_CENTER_Y = '/robot1/tracked_target/center_y'      # std_msgs/Float32
TOPIC_DEPTH_M = '/robot1/tracked_target/depth_m'        # std_msgs/Float32

# ---------------------------------------------------------------------
# 화면에 표시/추종 대상으로 인정할 객체 조건 (navigation.py의 target_label,
# min_confidence와 반드시 동일하게 맞춰야 합니다).
# ---------------------------------------------------------------------
TARGET_LABEL = 'orange'
MIN_CONFIDENCE = 0.7


class OakdViewer(Node):
    def __init__(self):
        super().__init__('oakd_viewer')

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None  # uint16, 단위 mm

        self.get_logger().info(f'YOLO 모델 로딩 중: {YOLO_MODEL_PATH}')
        self.yolo_model = YOLO(YOLO_MODEL_PATH)
        self.get_logger().info('YOLO 모델 로딩 완료')

        self.window_name = 'Yolo Detect'

        # /robot1/events/yolo_tracker_enable 신호로 활성/비활성을 토글합니다.
        # (1회성 트리거가 아니라 반복적으로 True/False를 받을 수 있습니다.)
        self.activated = False

        # 카메라 구독/타이머/발행자는 activate()에서 생성되고 deactivate()에서 정리됩니다
        # (초기값만 None으로 선언).
        self.rgb_subscriber = None
        self.depth_subscriber = None
        self.display_timer = None
        self.detected_publisher = None
        self.label_publisher = None
        self.confidence_publisher = None
        self.center_x_publisher = None
        self.center_y_publisher = None
        self.depth_m_publisher = None

        # 터미널 로그용 상태 추적 변수입니다.
        # frame_count: 지금까지 RGB 콜백으로 받은 프레임 수 (이미지 캡처 확인용).
        # last_rgb_log_count: 캡처 성공 로그를 너무 자주 찍지 않기 위한 카운터입니다.
        self.frame_count = 0
        self.rgb_log_interval = 30  # 약 1초(30fps 기준)마다 1번 캡처 로그를 남깁니다.
        # detected_state: 직전 프레임의 검출 성공/실패 상태. 상태가 바뀔 때만 로그를 남겨
        # 매 프레임(30fps)마다 터미널이 도배되는 것을 막습니다.
        self.detected_state = None

        # enable 신호 구독 (활성화/비활성화 토글). 노드가 살아있는 동안 계속 구독합니다.
        self.enable_subscriber = self.create_subscription(
            Bool, YOLO_TRACKER_ENABLE_TOPIC, self.enable_callback, 10,
        )

        self.get_logger().info(
            f"oakd_viewer 대기 중... '{YOLO_TRACKER_ENABLE_TOPIC}' 신호(True)를 기다립니다."
        )

    # ------------------------------------------------------------------
    # 활성화/비활성화 토글
    # ------------------------------------------------------------------
    def enable_callback(self, msg: Bool):
        if msg.data:
            if self.activated:
                return  # 이미 활성 상태면 중복 활성화하지 않음
            self.activated = True
            self.get_logger().info(
                f"'{YOLO_TRACKER_ENABLE_TOPIC}' True 신호 수신. OAK-D YOLO viewer를 시작합니다."
            )
            self.activate()
        else:
            if not self.activated:
                return  # 이미 비활성 상태면 무시
            self.activated = False
            self.get_logger().info(
                f"'{YOLO_TRACKER_ENABLE_TOPIC}' False 신호 수신. OAK-D YOLO viewer를 정지합니다."
            )
            self.deactivate()

    def activate(self):
        """enable 신호(True)를 받은 시점에 창/카메라 구독/타이머/발행자를 시작한다."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.rgb_subscriber = self.create_subscription(
            CompressedImage,
            'robot1/oakd/rgb/image_raw/compressed',
            self.rgb_callback,
            qos_profile_sensor_data,
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            'robot1/oakd/stereo/image_raw',
            self.stereo_callback,
            qos_profile_sensor_data,
        )

        # 디스플레이 갱신용 타이머 (메인 스레드에서 cv2.imshow/waitKey 실행)
        self.display_timer = self.create_timer(1.0 / 30.0, self.update_display)

        # navigation node로 보내는 tracked_target 필드별 발행자 (6개, std_msgs 표준 타입)
        self.detected_publisher = self.create_publisher(Bool, TOPIC_DETECTED, 10)
        self.label_publisher = self.create_publisher(String, TOPIC_LABEL, 10)
        self.confidence_publisher = self.create_publisher(Float32, TOPIC_CONFIDENCE, 10)
        self.center_x_publisher = self.create_publisher(Float32, TOPIC_CENTER_X, 10)
        self.center_y_publisher = self.create_publisher(Float32, TOPIC_CENTER_Y, 10)
        self.depth_m_publisher = self.create_publisher(Float32, TOPIC_DEPTH_M, 10)

        self.get_logger().info('OAK-D YOLO viewer 시작됨. 검출 객체 중심 좌표/거리가 좌측 상단에 표시됩니다.')

    def deactivate(self):
        """enable 신호(False)를 받은 시점에 구독자/타이머/창/발행자를 정리하고 대기 상태로 돌아간다."""
        # 타이머를 먼저 멈춰 더 이상 update_display가 호출되지 않게 합니다.
        if self.display_timer is not None:
            self.display_timer.cancel()
            self.destroy_timer(self.display_timer)
            self.display_timer = None

        if self.rgb_subscriber is not None:
            self.destroy_subscription(self.rgb_subscriber)
            self.rgb_subscriber = None

        if self.depth_subscriber is not None:
            self.destroy_subscription(self.depth_subscriber)
            self.depth_subscriber = None

        if self.detected_publisher is not None:
            self.destroy_publisher(self.detected_publisher)
            self.detected_publisher = None
        if self.label_publisher is not None:
            self.destroy_publisher(self.label_publisher)
            self.label_publisher = None
        if self.confidence_publisher is not None:
            self.destroy_publisher(self.confidence_publisher)
            self.confidence_publisher = None
        if self.center_x_publisher is not None:
            self.destroy_publisher(self.center_x_publisher)
            self.center_x_publisher = None
        if self.center_y_publisher is not None:
            self.destroy_publisher(self.center_y_publisher)
            self.center_y_publisher = None
        if self.depth_m_publisher is not None:
            self.destroy_publisher(self.depth_m_publisher)
            self.depth_m_publisher = None

        # cv2.destroyWindow()/destroyAllWindows()만 호출하면 OpenCV의 GUI
        # 이벤트 루프가 그 시점에 처리되지 않아 창이 화면에 그대로 남는 경우가
        # 있습니다 (특히 Qt/GTK 백엔드). display_timer를 이미 멈췄기 때문에 그
        # 이후로 waitKey가 호출될 기회가 없으므로, destroy 호출 직후 waitKey를
        # 여러 번 호출하면서 짧게 대기해 GUI 이벤트 루프가 창을 실제로 닫을
        # 시간을 충분히 주도록 합니다. destroyWindow보다 destroyAllWindows가
        # 더 확실하게 백엔드 리소스를 정리합니다.
        cv2.destroyAllWindows()
        for _ in range(10):
            cv2.waitKey(1)
            time.sleep(0.01)

        # 다음 활성화 시 이전 프레임이 섞이지 않도록 캐시/상태를 초기화합니다
        # (프로세스를 종료하지 않으므로 메모리에 남아있던 이전 이미지를 직접 비워줘야 합니다).
        self.rgb_image = None
        self.depth_image = None
        self.frame_count = 0
        self.detected_state = None

        self.get_logger().info(
            f"OAK-D YOLO viewer 정지됨. '{YOLO_TRACKER_ENABLE_TOPIC}' 신호(True)를 다시 기다립니다."
        )

    # ------------------------------------------------------------------
    # 콜백들
    # ------------------------------------------------------------------
    def rgb_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.rgb_image = img

            # 이미지 캡처(수신+디코딩)가 정상적으로 이루어지고 있음을 주기적으로 로그로 남깁니다.
            # 매 프레임마다 찍으면 터미널이 도배되므로 rgb_log_interval 프레임마다 1번만 출력합니다.
            self.frame_count += 1
            if self.frame_count % self.rgb_log_interval == 0:
                h, w = img.shape[:2]
                self.get_logger().info(
                    f"이미지 캡처 성공: frame={self.frame_count}, size={w}x{h}"
                )
        else:
            # imdecode가 실패하면(손상된 압축 데이터 등) 캡처 실패로 보고 경고를 남깁니다.
            self.get_logger().warn("이미지 캡처 실패: imdecode 결과가 None입니다.")

    def stereo_callback(self, msg: Image):
        try:
            # 16UC1, 단위 mm
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'stereo 이미지 변환 실패: {e}')
            return

        self.depth_image = depth

    # ------------------------------------------------------------------
    # 거리 조회
    # ------------------------------------------------------------------
    def get_distance_mm(self, rgb_x, rgb_y):
        """RGB 이미지 좌표(rgb_x, rgb_y)에 대응하는 depth 값을 mm 단위로 반환."""
        if self.depth_image is None or self.rgb_image is None:
            return None

        rh, rw = self.rgb_image.shape[:2]
        dh, dw = self.depth_image.shape[:2]

        # RGB와 depth 해상도가 다르면 비율로 좌표 변환
        dx = int(rgb_x * dw / rw)
        dy = int(rgb_y * dh / rh)

        if dx < 0 or dy < 0 or dx >= dw or dy >= dh:
            return None

        # 노이즈 영향을 줄이기 위해 3x3 윈도우에서 0이 아닌 값들의 중앙값 사용
        half = 1
        x0, x1 = max(0, dx - half), min(dw, dx + half + 1)
        y0, y1 = max(0, dy - half), min(dh, dy + half + 1)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[patch > 0]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def get_rgb_center(self):
        """RGB 이미지 기준 중앙 좌표(검출 없을 때 fallback용)."""
        rh, rw = self.rgb_image.shape[:2]
        return rw // 2, rh // 2

    # ------------------------------------------------------------------
    # 화면 갱신
    # ------------------------------------------------------------------
    def update_display(self):
        # deactivate()가 display_timer.cancel()을 호출해도, 호출 시점에 이미
        # executor 큐에 들어가 있던 update_display 실행 건은 한 번 더 처리될 수
        # 있습니다. 그 경우 cv2.destroyWindow() 직후에 다시 imshow가 호출되어
        # 창이 재생성되는 것처럼 보이는 문제가 생기므로, 비활성 상태에서는
        # 절대 그리지 않도록 여기서 한 번 더 막습니다.
        if not self.activated:
            return

        if self.rgb_image is None:
            return

        # YOLO 추론 (원본 해상도 기준)
        results = self.yolo_model(self.rgb_image, verbose=False)[0]

        target_x, target_y = self.get_rgb_center()  # 검출 없을 때 fallback
        best_conf = -1.0
        best_label = None
        boxes_to_draw = []  # (x1, y1, x2, y2, label, conf)

        if results.boxes is not None and len(results.boxes) > 0:
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.yolo_model.names.get(cls_id, str(cls_id))

                # TARGET_LABEL(orange)이고 MIN_CONFIDENCE 이상인 객체만
                # 화면 표시/추종 후보로 인정합니다. navigation.py의 필터 조건과 동일합니다.
                if label.lower() != TARGET_LABEL or conf < MIN_CONFIDENCE:
                    continue

                boxes_to_draw.append((x1, y1, x2, y2, label, conf))

                if conf > best_conf:
                    best_conf = conf
                    best_label = label
                    target_x = int((x1 + x2) / 2)
                    target_y = int((y1 + y2) / 2)

        # YOLO 추론이 정상적으로 수행되어 결과를 얻었음을 알리는 로그입니다.
        # 검출 성공 <-> 실패 상태가 바뀐 시점에만 출력해 매 프레임 도배를 방지합니다.
        detected_now = best_label is not None
        if detected_now != self.detected_state:
            if detected_now:
                self.get_logger().info(
                    f"YOLO 처리 성공: '{best_label}' 검출됨 (confidence={best_conf:.2f}, "
                    f"center=({target_x}, {target_y}))"
                )
            else:
                self.get_logger().info(
                    f"YOLO 처리 완료: 조건(label={TARGET_LABEL}, conf>={MIN_CONFIDENCE})을 "
                    f"만족하는 객체가 없습니다."
                )
            self.detected_state = detected_now

        distance_mm = self.get_distance_mm(target_x, target_y)

        coord_text = f'x: {target_x}, y: {target_y}'
        if distance_mm is not None:
            dist_text = f'dist: {distance_mm / 1000.0:.2f} m'
        else:
            dist_text = 'dist: N/A'

        # confidence 표시용 텍스트입니다. 검출된 객체가 없으면 N/A로 표시합니다.
        if best_label is not None:
            conf_text = f'conf: {best_conf:.2f} ({best_label})'
        else:
            conf_text = 'conf: N/A'

        # 화면 표시용으로 먼저 4.0배 확대 (텍스트/마커/박스가 같이 또렷하게 커지도록 확대 후 그림)
        scale = 4.0
        display = cv2.resize(
            self.rgb_image, None,
            fx=scale, fy=scale,
            interpolation=cv2.INTER_LINEAR,
        )

        # YOLO 검출 박스 그리기
        for x1, y1, x2, y2, label, conf in boxes_to_draw:
            p1 = (int(x1 * scale), int(y1 * scale))
            p2 = (int(x2 * scale), int(y2 * scale))
            cv2.rectangle(display, p1, p2, (0, 255, 0), 3)
            cv2.putText(
                display, f'{label} {conf:.2f}', (p1[0], max(0, p1[1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA,
            )

        scaled_tx, scaled_ty = int(target_x * scale), int(target_y * scale)

        # 기준 좌표(검출 중심 또는 화면 중앙)에 십자 마커 표시
        cv2.drawMarker(
            display, (scaled_tx, scaled_ty), (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=35,
            thickness=5,
        )

        # 좌측 상단: 좌표/거리/confidence
        self.draw_overlay_text(display, [coord_text, dist_text, conf_text])

        # navigation node로 검출 결과 발행 (필드별 6개 토픽)
        self.publish_tracked_target(best_label, best_conf, target_x, target_y, distance_mm)

        cv2.imshow(self.window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("'q' 입력됨. 노드를 종료합니다.")
            rclpy.shutdown()

    @staticmethod
    def draw_overlay_text(img, lines, origin=(25, 60), line_gap=70):
        """좌측 상단에 배경 박스와 함께 여러 줄 텍스트를 그려준다."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 1.75
        thickness = 5

        x0, y0 = origin
        box_w = max(cv2.getTextSize(t, font, scale, thickness)[0][0] for t in lines) + 50
        box_h = line_gap * len(lines) + 25

        overlay = img.copy()
        cv2.rectangle(overlay, (x0 - 25, y0 - 50), (x0 - 25 + box_w, y0 - 50 + box_h),
                       (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)

        for i, text in enumerate(lines):
            y = y0 + i * line_gap
            cv2.putText(img, text, (x0, y), font, scale, (0, 255, 0), thickness, cv2.LINE_AA)

    # ------------------------------------------------------------------
    # navigation node로 검출 결과 발행 (필드별 독립 토픽, std_msgs 표준 타입)
    # ------------------------------------------------------------------
    def publish_tracked_target(self, label, confidence, center_x, center_y, distance_mm):
        target = label  # label이 None이면 검출된 객체가 없었다는 뜻

        if target is None:
            # 탐지되지 않은 경우에도 모든 필드에 기본값을 발행한다.
            detected = False
            label = ""
            confidence = 0.0
            center_x = 0.0
            center_y = 0.0
            depth_m = 0.0
        else:
            detected = True
            confidence = round(float(confidence), 2)
            center_x = float(center_x)
            center_y = float(center_y)
            depth_m = round(distance_mm / 1000.0, 2) if distance_mm is not None else 0.0

        self.detected_publisher.publish(Bool(data=detected))
        self.label_publisher.publish(String(data=label))
        self.confidence_publisher.publish(Float32(data=float(confidence)))
        self.center_x_publisher.publish(Float32(data=float(center_x)))
        self.center_y_publisher.publish(Float32(data=float(center_y)))
        self.depth_m_publisher.publish(Float32(data=float(depth_m)))


def main(args=None):
    rclpy.init(args=args)
    node = OakdViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()