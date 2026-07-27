#!/usr/bin/env python3
# ===================================================================
# AMR2 제어 노드 (단일 노드, status 단계 기반)
#
# [역할] System Monitor의 control_signal을 받아 단계별로
#        정해진 신호만 받아들이고 액션(언독/주행/도킹)을 수행
#
# [단계 (status)]
#   WAIT_GO     : 도킹 대기   → go==True 만 받음 (언독)
#   WAIT_ROOM   : 언독 완료   → room_id!=0 만 받음 (병실 주행)
#   WAIT_RETURN : 병실 도착   → go==False 만 받음 (복귀+도킹)
#   BUSY        : 작업 중     → 모든 신호 무시
#
# [메시지] amr2_control_interface/Amr2ControlSignal { bool go, int64 room_id }
# [토픽]   /amr2/control_signal
#
# [자동 init_pose]
#   노드 시작 시 도킹 스테이션 좌표를 AMCL에 자동으로 발행하여
#   RViz의 2D Pose Estimate 수동 입력 없이 위치 초기화
# ===================================================================
 
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from amr2_control_interface.msg import Amr2ControlSignal
 
 
class Amr2Controller(Node):
    def __init__(self):
        super().__init__('amr2_controller')
 
        NS = 'robot5'
        nav_action  = f'/{NS}/navigate_to_pose'
        dock_action = f'/{NS}/dock'
        undock_act  = f'/{NS}/undock'
        dock_status_topic = f'/{NS}/dock_status'
        initialpose_topic = f'/{NS}/initialpose'
 
        # ── 단계 상태 변수 ──
        self.status = 'WAIT_GO'
 
        # ── 로봇 실제 도킹 상태 (dock_status 토픽으로 갱신) ──
        self.is_docked = True
 
        # ── 신호 구독 ──
        self.sig_sub = self.create_subscription(
            Amr2ControlSignal, '/amr2/control_signal', self.on_signal, 10)
 
        # ── dock_status 구독 (실제 도킹 여부 확인용) ──
        self.dock_status_sub = self.create_subscription(
            DockStatus, dock_status_topic, self.dock_status_cb, 10)
 
        # ── AMCL initialpose 발행자 (자동 초기 위치 설정) ──
        # initialpose는 transient_local + reliable QoS로 발행해야 AMCL이 받음
        initpose_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, initialpose_topic, initpose_qos)
 
        # ── 액션 클라이언트 ──
        self.nav_client    = ActionClient(self, NavigateToPose, nav_action)
        self.undock_client = ActionClient(self, Undock,         undock_act)
        self.dock_client   = ActionClient(self, Dock,           dock_action)
 
        # ===========================================================
        # ★ 좌표 설정 ★
        # ===========================================================
 
        # ── [INIT_POSE] 로봇이 시작하는 실제 위치 = 도킹 스테이션 좌표 ──
        # 도크에 결합된 상태에서 amcl_pose를 측정해 여기에 넣어야 함
        # (RViz로 2D Pose Estimate 잡았던 그 위치)
        # 임시값 — 실측 후 교체 필요
        self.INIT_X  = 0.002786729      # ← TODO: 실측값으로 교체
        self.INIT_Y  =    -0.227313    # ← TODO: 실측값으로 교체
        self.INIT_QZ =  -0.0099      # ← TODO: 실측값으로 교체
        self.INIT_QW =  0.99951    # ← TODO: 실측값으로 교체
 
        # ── 병실 좌표 (room_id → 좌표) ──
        self.ROOM_COORDS = {
            101: self.make_pose(-3.3,  2.7, -0.9,   0.099),
            102: self.make_pose(-3.4,  3.7,  -0.9,   0.09),
            103: self.make_pose(-3.53, 4.8,  -0.9,   0.03),
        }
 
        # ── 홈(도킹 스테이션 근처) 좌표 ── 도크에서 살짝 떨어진 곳
        self.home_pose = self.make_pose(-0.6547, -0.7819, 0.4215, 0.906)
 
        # ── 중간 대기 지점 (staging) ── 언독 직후 벽 회피용 안전한 지점
        # ← TODO: 실측값으로 교체
        self.staging_pose = self.make_pose(x=-0.698, y=-0.800, qz=0.974, qw=0.2244)
 
        # 주행 후 동작 ('staging', 'room' 또는 'home')
        self.purpose = None
 
        # 도킹 재시도 카운트
        self._dock_retry = 0
        self.MAX_DOCK_RETRY = 3
 
        # ── 자동 초기 위치 발행 (재시도 + AMCL 수렴 확인) ──
        # AMCL이 구독자로 붙을 시간을 주기 위해 3초 후 첫 발행, 2초 간격 재시도
        amcl_pose_topic = f'/{NS}/amcl_pose'
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, amcl_pose_topic, self._amcl_pose_cb, 10)
        self._amcl_received = False
        self._init_attempt = 0
        self.MAX_INIT_ATTEMPTS = 15
        self.INIT_INTERVAL = 2.0
        self._init_timer = self.create_timer(3.0, self._publish_init_pose)
        self.get_logger().info('AMR2 제어 노드 가동! status=WAIT_GO (도킹 대기)')
        self.get_logger().info('AMCL 활성화 대기 중... (init_pose 자동 발행 예정)')

    # ---------------------------------------------------------------
    # [콜백] AMCL pose 수신 시 init_pose 발행 중단
    # ---------------------------------------------------------------
    def _amcl_pose_cb(self, msg):
        if not self._amcl_received:
            self._amcl_received = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(
                f'AMCL 수렴 확인! pose=({x:.2f}, {y:.2f}) — init_pose 자동 발행 중단')
 
    # ---------------------------------------------------------------
    # [헬퍼] PoseStamped 생성 (Navigate goal용)
    # ---------------------------------------------------------------
    def make_pose(self, x, y, qz, qw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = float(qz)
        p.pose.orientation.w = float(qw)
        return p
 
    # ---------------------------------------------------------------
    # [자동 INIT POSE] AMCL에 초기 위치 자동 발행 (재시도 + 정규화)
    # ---------------------------------------------------------------
    def _publish_init_pose(self):
        """AMCL이 받을 때까지 2초마다 재시도 발행."""
        # 이미 AMCL이 응답했으면 발행 중단
        if self._amcl_received:
            if self._init_timer is not None:
                self._init_timer.cancel()
                self._init_timer = None
            return

        self._init_attempt += 1
        if self._init_attempt > self.MAX_INIT_ATTEMPTS:
            self.get_logger().error(
                f'init_pose 자동 발행 실패 ({self.MAX_INIT_ATTEMPTS}회 시도 후 '
                'AMCL 응답 없음). AMCL 노드 확인 필요. '
                'RViz로 2D Pose Estimate 수동 설정 가능.')
            self._init_timer.cancel()
            return

        # quaternion 4개 모두 정규화 (malformed 방지)
        qx = 0.0
        qy = 0.0
        qz = float(self.INIT_QZ)
        qw = float(self.INIT_QW)
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-6:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.INIT_X)
        msg.pose.pose.position.y = float(self.INIT_Y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 공분산 (RViz 기본값)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0]  = 0.25   # x
        msg.pose.covariance[7]  = 0.25   # y
        msg.pose.covariance[35] = 0.0685 # yaw

        self.initpose_pub.publish(msg)
        self.get_logger().info(
            f'init_pose 발행 시도 {self._init_attempt}/{self.MAX_INIT_ATTEMPTS} '
            f'(x={self.INIT_X}, y={self.INIT_Y}, qz={qz:.4f}, qw={qw:.4f})')

        # 첫 발행 후엔 2초 간격으로 재시도
        if self._init_attempt == 1:
            self._init_timer.cancel()
            self._init_timer = self.create_timer(
                self.INIT_INTERVAL, self._publish_init_pose)
 
    # ---------------------------------------------------------------
    # [콜백] dock_status — 로봇 실제 도킹 여부 갱신
    # ---------------------------------------------------------------
    def dock_status_cb(self, msg):
        self.is_docked = msg.is_docked
 
    # ===============================================================
    # [핵심] 신호 수신 — 단계별로 정해진 신호만 받음
    # ===============================================================
    def on_signal(self, msg):
        go = msg.go
        room_id = msg.room_id
 
        # ── WAIT_GO : go==True 만 받음 (언독) ──
        if self.status == 'WAIT_GO':
            if go == True:
                self.get_logger().info('[WAIT_GO] GO 수신 → 언독 시작')
                self.status = 'BUSY'
                self.start_undock()
            else:
                self.get_logger().warn(f'[WAIT_GO] GO만 가능. 무시 (go={go})')
 
        # ── WAIT_ROOM : room_id!=0 만 받음 (병실 주행) ──
        elif self.status == 'WAIT_ROOM':
            if go == True and room_id != 0:
                if room_id in self.ROOM_COORDS:
                    self.get_logger().info(f'[WAIT_ROOM] room_id={room_id} 수신 → 병실 주행')
                    self.status = 'BUSY'
                    self.start_navigate(self.ROOM_COORDS[room_id], purpose='room')
                else:
                    self.get_logger().warn(f'[WAIT_ROOM] 등록되지 않은 room_id={room_id}. 무시')
            else:
                self.get_logger().warn(f'[WAIT_ROOM] room_id만 가능. 무시 (go={go}, room_id={room_id})')
 
        # ── WAIT_RETURN : go==False 만 받음 (복귀) ──
        elif self.status == 'WAIT_RETURN':
            if go == False:
                self.get_logger().info('[WAIT_RETURN] RETURN 수신 → 복귀 시작')
                self.status = 'BUSY'
                self.start_navigate(self.home_pose, purpose='home')
            else:
                self.get_logger().warn(f'[WAIT_RETURN] RETURN만 가능. 무시 (go={go})')
 
        # ── BUSY : 작업 중, 모두 무시 ──
        else:
            self.get_logger().warn(f'[BUSY] 작업 중. 무시 (go={go}, room_id={room_id})')
 
    # ===============================================================
    # 1) 언독
    # ===============================================================
    def start_undock(self):
        if not self.undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('undock 서버 없음')
            self.status = 'WAIT_GO'
            return
        self.undock_client.send_goal_async(Undock.Goal()).add_done_callback(self.undock_resp_cb)
 
    def undock_resp_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('언독 거절 → 그래도 병실 선택 대기로')
            self.status = 'WAIT_ROOM'
            return
        gh.get_result_async().add_done_callback(self.undock_result_cb)
 
    def undock_result_cb(self, future):
        self.get_logger().info('언독 완료! 중간 대기 지점으로 이동 시작')
        self._go_to_staging()
 
    # ---------------------------------------------------------------
    # [헬퍼] staging 지점으로 이동 (벽 회피용)
    # ---------------------------------------------------------------
    def _go_to_staging(self):
        self.start_navigate(self.staging_pose, purpose='staging')
 
    # ===============================================================
    # 2) 주행 (병실/홈 공통)
    # ===============================================================
    def start_navigate(self, pose, purpose):
        self.purpose = purpose
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate 서버 없음')
            self._recover()
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav_client.send_goal_async(goal).add_done_callback(self.nav_resp_cb)
 
    def nav_resp_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('주행 거절')
            self._recover()
            return
        gh.get_result_async().add_done_callback(self.nav_result_cb)
 
    def nav_result_cb(self, future):
        status = future.result().status
        if status != 4:                 # 4 = SUCCEEDED
            self.get_logger().error(f'주행 실패 (코드 {status})')
            self._recover()
            return
 
        # ★ 도착(ARRIVED) = 주행 액션 SUCCEEDED
        if self.purpose == 'staging':
            self.status = 'WAIT_ROOM'    # staging 도착 → 병실 선택 대기
            self.get_logger().info('중간 대기 지점 도착! status=WAIT_ROOM (병실 선택 대기)')
        elif self.purpose == 'room':
            self.status = 'WAIT_RETURN'  # 병실 도착 → 복귀 대기
            self.get_logger().info('병실 도착! status=WAIT_RETURN (복귀 명령 대기)')
        elif self.purpose == 'home':
            self.get_logger().info('홈 도착! 도킹 시작')
            self._dock_retry = 0
            self.start_dock()
 
    # ===============================================================
    # 3) 도킹  (★ 결과 검증 + dock_status 확인 + 재시도 추가)
    # ===============================================================
    def start_dock(self):
        if self.is_docked:
            self.get_logger().info('이미 도킹 상태 → 완료 처리')
            self._dock_done()
            return
 
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('dock 서버 없음')
            self._recover()
            return
        self.get_logger().info(f'도킹 시도... ({self._dock_retry + 1}/{self.MAX_DOCK_RETRY})')
        self.dock_client.send_goal_async(Dock.Goal()).add_done_callback(self.dock_resp_cb)
 
    def dock_resp_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('도킹 Goal 거절됨')
            self._retry_dock()
            return
        gh.get_result_async().add_done_callback(self.dock_result_cb)
 
    def dock_result_cb(self, future):
        status = future.result().status
        self.get_logger().info(f'도킹 액션 결과 status={status}, is_docked={self.is_docked}')
 
        if status == 4 and self.is_docked:
            self._dock_done()
        elif self.is_docked:
            self.get_logger().info('실제 도킹 확인됨 (is_docked=True)')
            self._dock_done()
        else:
            self.get_logger().warn(f'도킹 실패 (status={status}, is_docked={self.is_docked})')
            self._retry_dock()
 
    def _retry_dock(self):
        self._dock_retry += 1
        if self._dock_retry < self.MAX_DOCK_RETRY:
            self.get_logger().warn(f'도킹 재시도 {self._dock_retry}/{self.MAX_DOCK_RETRY}')
            self._dock_timer = self.create_timer(2.0, self._dock_retry_cb)
        else:
            self.get_logger().error(
                '도킹 반복 실패. home_pose가 도크에서 멀거나 방향이 안 맞을 수 있음. '
                '복귀 대기로 되돌림.')
            self.status = 'WAIT_RETURN'
 
    def _dock_retry_cb(self):
        self._dock_timer.cancel()
        self.start_dock()
 
    def _dock_done(self):
        self.status = 'WAIT_GO'         # ★ 도킹 완료 → 처음으로 (루프)
        self.get_logger().info('도킹 완료! status=WAIT_GO (임무 종료, 대기)')
 
    # ===============================================================
    # 복구: 실패 시 직전 대기 단계로 되돌림
    # ===============================================================
    def _recover(self):
        if self.purpose == 'staging':
            self.status = 'WAIT_GO'      # staging 실패 → 처음으로
        elif self.purpose == 'room':
            self.status = 'WAIT_ROOM'
        elif self.purpose == 'home':
            self.status = 'WAIT_RETURN'
        else:
            self.status = 'WAIT_GO'
        self.get_logger().warn(f'복구 → status={self.status}')
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Amr2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()