import sqlite3
from datetime import datetime, timedelta
import threading  
import json  
import time
import os

from flask import Flask, render_template, request, jsonify
# Flask-SocketIO 라이브러리 임포트
from flask_socketio import SocketIO, emit

# ROS 2 관련 라이브러리 임포트
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped 
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# AMR2 제어용 커스텀 메시지 타입 임포트
try:
    from amr2_control_interface.msg import Amr2ControlSignal
except ImportError:
    Amr2ControlSignal = None

# 표준 배터리 메시지 타입 임포트
try:
    from sensor_msgs.msg import BatteryState
except ImportError:
    BatteryState = None



BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DB_NAME = os.path.join(BASE_DIR, 'mydatabase.db')

app = Flask(__name__, 
            static_folder=os.path.join(BASE_DIR, 'static'), 
            template_folder=os.path.join(BASE_DIR, 'templates'))
app.config['SECRET_KEY'] = 'secret_hospital_key!'
socketio = SocketIO(app, cors_allowed_origins="*")

# 🔍 [여기에 추가] 서버 켜질 때 진짜로 파일이 매칭되는지 눈으로 확인하는 검증용 코드
target_map_path = os.path.join(app.static_folder, 'map.png')
if os.path.exists(target_map_path):
    print(f"✅ [SYSTEM CHECK] 지도 이미지 감지 성공! -> {target_map_path}", flush=True)
else:
    print(f"❌ [SYSTEM CHECK] 경고! static 폴더 안에 'map.png'가 없습니다! -> 기대 경로: {target_map_path}", flush=True)



# 내부에서 타이머 계산 및 메타데이터 동기화를 위한 최소한의 데이터 구조 유지
status_lock = threading.Lock()
# ==========================================
# [수정] amr1과 amr2의 초기 좌표를 올바르게 매칭
# ==========================================
status_lock = threading.Lock()
robot_status = {
    'amr1': {
        'battery': '-',
        'x': -4.598237895965576, 'y': 5.571274089813232,               # 📍 첫 번째 RViz 2D Pose Estimate 로그 (amr1 자체 맵 프레임)
        'map_origin_x': -5.39, 'map_origin_y': -1.62,  # 📍 amr1 전용 map.yaml origin (amr2 맵과 다른 프레임)
        'map_resolution': 0.05,                        
        'remaining_time': '대기 중', 'remaining_seconds': 0, 'timer_running': False,
        'fluid_name': '', 'fluid_volume': 0, 'last_updated': 0         
    },
    'amr2': {
        'battery': '-',
        'x': 0, 'y': 0,                   # 📍 두 번째 RViz 2D Pose Estimate 로그 (표시 지도와 동일 프레임)
        'map_origin_x': -5.76, 'map_origin_y': -1.79,  # 📍 amr2 = 웹 배경 표시 지도 map.yaml origin
        'map_resolution': 0.05,
        'last_updated': 0         
    }
}


ros_node_instance = None


# ==========================================
# [ROS 2 Publisher & Subscriber 내장 노드 정의]
# ==========================================
class RobotMonitorNode(Node):
    def __init__(self):
        super().__init__('flask_robot_monitor_node')

        battery_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        timer_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publisher 정의
        self.timer_pub = self.create_publisher(String, '/amr1/infusion_timer', 10)

        # AMR1 신호 보내기용 publisher 생성 
        self.timer_done_pub = self.create_publisher(Bool, '/robot1/events/iv_timer_done', timer_qos)

        # AMR2 명령용 주행 제어 커스텀 토픽 Publisher 객체 생성
        if Amr2ControlSignal is not None:
            self.amr2_command_pub = self.create_publisher(Amr2ControlSignal, '/amr2/control_signal', 10)
            self.get_logger().info("✅ [AMR2] 커스텀 메시지(Amr2ControlSignal) 퍼블리셔 등록 완료")

        # Subscriber 정의
        if BatteryState is not None:
            self.amr1_battery_sub = self.create_subscription(
                BatteryState, '/robot1/battery_state', self.r1_standard_bat_cb, battery_qos)
        
        
        if BatteryState is not None:
            self.amr2_battery_sub = self.create_subscription(
                BatteryState, '/robot5/battery_state', self.r5_standard_bat_cb, battery_qos)


        # self.amr1_pose_sub = self.create_subscription(
                #     PoseWithCovarianceStamped, '/robot1/amcl_pose', self.r1_amcl_pose_cb, pose_qos)

        self.amr2_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot5/amcl_pose', self.r5_amcl_pose_cb, pose_qos)

        self.sub_monitor1_call = self.create_subscription(
            Bool, '/monitor1/nurse_call', self.monitor1_call_callback, 10)

        self.get_logger().info("🚀 Central System Monitor Node 활성화 (SocketIO 실시간 연동)")

    # ==========================================
    # ROS 콜백 함수들
    # ==========================================

    def r1_standard_bat_cb(self, msg):
        try:
            if msg is None: return
            raw_val = msg.percentage
            val = int(raw_val * 100) if raw_val <= 1.0 else int(raw_val)
            val = max(0, min(100, val))
            
            self.get_logger().info(f"📥 [AMR1 수신] 배터리 수량: {val}%")
            
            timestamp = int(time.time())
            with status_lock:
                robot_status['amr1']['battery'] = str(val)
                robot_status['amr1']['last_updated'] = timestamp
            
            socketio.emit('robot_update', {
                'robot': 'amr1', 
                'type': 'battery', 
                'value': str(val), 
                'last_updated': timestamp
            })
        except Exception as e:
            self.get_logger().error(f"⚠️ [Robot 1 배터리] 에러: {e}")

    def r1_amcl_pose_cb(self, msg):
        try:
            if msg is None: return
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation  
            
            self.get_logger().info(
                f"🗺️ [AMR1 AMCL Pose 수신] -> X: {position.x:.4f}m, Y: {position.y:.4f}m, Z_ori: {orientation.z:.4f}"
            )
            
            timestamp = int(time.time())
            with status_lock:
                robot_status['amr1']['x'] = position.x
                robot_status['amr1']['y'] = position.y
                robot_status['amr1']['last_updated'] = timestamp

            socketio.emit('robot_update', {
                'robot': 'amr1', 
                'type': 'pose', 
                'x': position.x, 
                'y': position.y,
                'last_updated': timestamp
            })
        except Exception as e:
            self.get_logger().error(f"⚠️ [Robot 1] AMCL Pose 에러: {e}")

    def r5_standard_bat_cb(self, msg):
        try:
            if msg is None: return
            raw_val = msg.percentage 
            val = int(raw_val * 100) if raw_val <= 1.0 else int(raw_val)
            val = max(0, min(100, val))
            
            self.get_logger().info(f"📥 [AMR2 수신] 배터리 수량: {val}%")
            
            timestamp = int(time.time())
            with status_lock:
                robot_status['amr2']['battery'] = str(val)
                robot_status['amr2']['last_updated'] = timestamp

            socketio.emit('robot_update', {
                'robot': 'amr2', 
                'type': 'battery', 
                'value': str(val), 
                'last_updated': timestamp
            })
        except Exception as e:
            self.get_logger().error(f"⚠️ [Robot 5 배터리] 에러: {e}")

    def r5_amcl_pose_cb(self, msg):
        try:
            if msg is None: return
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            self.get_logger().info(
                f"🗺️ [AMR2 AMCL Pose 수신] -> X: {position.x:.4f}m, Y: {position.y:.4f}m, Z_ori: {orientation.z:.4f}"
            )
            
            timestamp = int(time.time())
            with status_lock:
                robot_status['amr2']['x'] = position.x
                robot_status['amr2']['y'] = position.y
                robot_status['amr2']['last_updated'] = timestamp

            socketio.emit('robot_update', {
                'robot': 'amr2', 
                'type': 'pose', 
                'x': position.x, 
                'y': position.y,
                'last_updated': timestamp
            })
        except Exception as e:
            self.get_logger().error(f"⚠️ [Robot 5] AMCL Pose 에러: {e}")

    def monitor1_call_callback(self, msg):
        try:
            self.get_logger().warning(f"🚨 [Nurse Call 신호 수신] 호출 센서 상태: {msg.data}")
            called_at = datetime.now().strftime('%Y-%m-%d %H:%M:%S') if msg.data else None
            socketio.emit('nurse_call', {
                'monitor_id': 'monitor1', 'is_called': bool(msg.data), 'called_at': called_at
            })
        except Exception as e:
            self.get_logger().error(f"⚠️ [Monitor1] 호출 에러: {e}")

    def publish_timer_status(self):
        with status_lock:
            payload = {
                "remaining_seconds": robot_status['amr1']['remaining_seconds'],
                "timer_running": robot_status['amr1']['timer_running'],
                "fluid_name": robot_status['amr1']['fluid_name'],
                "fluid_volume": robot_status['amr1']['fluid_volume']
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.timer_pub.publish(msg)

    def publish_timer_done_event(self, status): 
        msg = Bool()
        msg.data = status
        self.timer_done_pub.publish(msg)

    def publish_amr2_command(self, go_bool, room_id_int=0):
        if Amr2ControlSignal is None:
            self.get_logger().error("Amr2ControlSignal 타입을 사용할 수 없습니다.")
            return
            
        msg = Amr2ControlSignal()
        msg.go = bool(go_bool)         
        msg.room_id = int(room_id_int) 
        
        self.get_logger().info(f"📤 [AMR2 ROS 토픽 발송] go: {msg.go}, room_id: {msg.room_id}")
        self.amr2_command_pub.publish(msg)


# ==========================================
# [중앙 제어형 타이머 백그라운드 워커 스레드]
# ==========================================
def central_timer_worker():
    global ros_node_instance
    while True:
        time.sleep(1)
        remaining_time_str = ""
        
        with status_lock:
            if robot_status['amr1']['timer_running'] and robot_status['amr1']['remaining_seconds'] > 0:
                robot_status['amr1']['remaining_seconds'] -= 1
                rem_sec = robot_status['amr1']['remaining_seconds']
                
                minutes = rem_sec // 60
                seconds = rem_sec % 60
                remaining_time_str = f"{minutes}분 {seconds}초 남음"
                robot_status['amr1']['remaining_time'] = remaining_time_str
                
                if rem_sec == 60:
                    if ros_node_instance is not None:
                        ros_node_instance.publish_timer_done_event(True)
                        print("📢 [SYSTEM] 수액 종료 1분 전 예고.", flush=True)
                
                elif rem_sec <= 0:
                    robot_status['amr1']['timer_running'] = False
                    robot_status['amr1']['remaining_seconds'] = 0
                    remaining_time_str = "0분 0초 남음"
                    
                    if ros_node_instance is not None:
                        ros_node_instance.publish_timer_done_event(False)
                        print("🚨 [SYSTEM] 수액 완전히 만료!", flush=True)
        
        if remaining_time_str:
            socketio.emit('robot_update', {
                'robot': 'amr1', 'type': 'timer', 'value': remaining_time_str
            })


# ==========================================
# [Flask 웹 백엔드 서버 및 API / SocketIO 핸들러]
# ==========================================
def get_db_connection():
    conn = sqlite3.connect(DB_NAME)
    conn.row_factory = sqlite3.Row
    return conn

def init_tables():
    with get_db_connection() as conn:
        cursor = conn.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS iv_infusion_records (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            patient_id TEXT NOT NULL,
            patient_name TEXT NOT NULL,
            nurse_id TEXT NOT NULL,
            nurse_name TEXT NOT NULL,
            fluid_name TEXT NOT NULL,
            fluid_volume REAL NOT NULL,
            recorded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            started_at TEXT,
            estimated_end_at TEXT
        );
        """)
        conn.commit()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/history-page')
def history_page():
    return render_template('history.html')

@socketio.on('connect')
def handle_connect():
    global ros_node_instance
    if ros_node_instance:
        ros_node_instance.get_logger().info("🔌 [SocketIO] 웹 클라이언트 연결됨.")
        
    # [추가된 부분] 웹 클라이언트가 처음 접속(새로고침)하면, 
    # 백엔드가 기억하고 있는 초기/최신 위치와 배터리를 즉시 프론트엔드에 쏴주어 마커 위치를 갱신시킴
    with status_lock:
        for r_id in ['amr1', 'amr2']:
            socketio.emit('robot_update', {
                'robot': r_id, 
                'type': 'pose', 
                'x': robot_status[r_id]['x'], 
                'y': robot_status[r_id]['y'],
                'last_updated': robot_status[r_id]['last_updated']
            })
            socketio.emit('robot_update', {
                'robot': r_id, 
                'type': 'battery', 
                'value': robot_status[r_id]['battery'], 
                'last_updated': robot_status[r_id]['last_updated']
            })
            
        socketio.emit('robot_update', {
            'robot': 'amr1', 
            'type': 'timer', 
            'value': robot_status['amr1']['remaining_time']
        })

@app.route('/api/calls/nurse-check', methods=['POST'])
def nurse_checked():
    global ros_node_instance
    try:
        data = request.json or {}
        monitor_id = data.get('monitor_id', 'monitor1')
        if ros_node_instance:
            ros_node_instance.get_logger().info(f"✅ [Nurse Call 해제] 확인 완료 ({monitor_id})")
        socketio.emit('nurse_call', {
            'monitor_id': monitor_id, 'is_called': False, 'called_at': None
        })
        return jsonify({'success': True})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


@app.route('/api/infusion', methods=['POST'])
def save_infusion():
    global ros_node_instance
    try:
        data = request.json or {}  
        patient_id = data.get('patient_id', 'P001')   
        nurse_id = data.get('nurse_id', 'N123')       
        fluid_name = data.get('fluid_name', '일반 식염수') 
        
        try:
            fluid_volume = float(data.get('fluid_volume', 0))
        except (ValueError, TypeError):
            return jsonify({'success': False, 'message': '용량 형식이 잘못되었습니다.'}), 400

        if fluid_volume <= 0:
            return jsonify({'success': False, 'message': '투약 용량을 선택하거나 입력해주세요.'}), 400

        patient_name = data.get('patient_name', '미지정 환자')   
        nurse_name = data.get('nurse_name', '미지정 간호사')       

        remaining_seconds = int(fluid_volume * 60)
        current_time = datetime.now()  
        estimated_end_time = current_time + timedelta(seconds=remaining_seconds)

        start_str = current_time.strftime('%Y-%m-%d %H:%M:%S')
        end_str = estimated_end_time.strftime('%Y-%m-%d %H:%M:%S')

        with get_db_connection() as conn:
            cursor = conn.cursor()
            insert_query = """
            INSERT INTO iv_infusion_records (patient_id, patient_name, nurse_id, nurse_name, fluid_name, fluid_volume, started_at, estimated_end_at) 
            VALUES (?, ?, ?, ?, ?, ?, ?, ?);
            """
            cursor.execute(insert_query, (patient_id, patient_name, nurse_id, nurse_name, fluid_name, fluid_volume, start_str, end_str))
            conn.commit()  

        minutes = remaining_seconds // 60
        seconds = remaining_seconds % 60
        remaining_time_str = f"{minutes}분 {seconds}초 남음"

        with status_lock:
            robot_status['amr1']['fluid_name'] = fluid_name
            robot_status['amr1']['fluid_volume'] = fluid_volume
            robot_status['amr1']['remaining_seconds'] = remaining_seconds
            robot_status['amr1']['timer_running'] = True      
            robot_status['amr1']['remaining_time'] = remaining_time_str

        if ros_node_instance:
            ros_node_instance.get_logger().info(f"💉 [원격 타이머 구동] 환자: {patient_name}, 설정시간: {fluid_volume}분")

        socketio.emit('robot_update', {
            'robot': 'amr1', 'type': 'timer', 'value': remaining_time_str
        })

        if rclpy.ok() and ros_node_instance:
            ros_node_instance.publish_timer_status()

        return jsonify({
            'success': True,
            'data': {
                'patient_name': patient_name,
                'nurse_name': nurse_name,
                'fluid_name': fluid_name,
                'fluid_volume': fluid_volume,
                'start_time': current_time.strftime('%H:%M:%S'),
                'end_time': estimated_end_time.strftime('%H:%M:%S')
            }
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


@app.route('/api/history', methods=['GET'])
def get_history():
    try:
        with get_db_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT patient_id, nurse_id, fluid_name, fluid_volume, started_at, estimated_end_at FROM iv_infusion_records ORDER BY id DESC")
            rows = cursor.fetchall()

            history_list = []
            for row in rows:
                history_list.append({
                    'patient_id': row['patient_id'],
                    'nurse_id': row['nurse_id'],
                    'fluid_name': row['fluid_name'], 
                    'fluid_volume': row['fluid_volume'],
                    'started_at': row['started_at'],
                    'estimated_end_at': row['estimated_end_at']
                })
        return jsonify(history_list)
    except Exception as e:
        return jsonify([]), 500


@app.route('/api/amr2/command', methods=['POST'])
def control_amr2():
    global ros_node_instance
    try:
        data = request.json or {}
        command = data.get('command')      
        room_id_val = data.get('room_id', 0) 
        
        if command not in ['GO', 'RETURN']:
            return jsonify({'success': False, 'message': '잘못된 명령 형식입니다.'}), 400
            
        if rclpy.ok() and ros_node_instance:
            # 📍 상단 try-except 블록에서 선언한 글로벌 Amr2ControlSignal 존재 여부 체크
            if Amr2ControlSignal is None:
                return jsonify({'success': False, 'message': 'Amr2ControlSignal 타입을 사용할 수 없습니다.'}), 500
                
            msg = Amr2ControlSignal()
            
            if command == 'RETURN':
                msg.go = False
                msg.room_id = 0
                msg_log = "🏡 충전 스테이션 복귀(RETURN)"
                
            elif command == 'GO':
                msg.go = True
                msg.room_id = int(room_id_val)
                
                if msg.room_id == 0:
                    msg_log = "🚀 도킹 해제 시작(Undock)"
                else:
                    msg_log = f"🏥 {msg.room_id}호 병실 주행 시작"

            ros_node_instance.get_logger().info(f"🕹️ [웹 관제 명령 전송] -> {msg_log} (go={msg.go}, room_id={msg.room_id})")
            ros_node_instance.amr2_command_pub.publish(msg)
            
            return jsonify({'success': True, 'message': f'{msg_log} 신호를 로봇에게 전달했습니다.'})
        else:
            return jsonify({'success': False, 'message': 'ROS 2 노드 비활성화 상태'}), 503
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 500


def run_ros2_node():
    global ros_node_instance
    ros_node_instance = RobotMonitorNode() 
    executor = rclpy.executors.MultiThreadedExecutor() 
    executor.add_node(ros_node_instance)
    try:
        executor.spin()
    except Exception as e:
        print(f"⚠️ ROS 2 종료: {e}", flush=True)
    finally:
        executor.shutdown()
        ros_node_instance.destroy_node()

if __name__ == '__main__':
    init_tables()
    rclpy.init()
    
    ros2_thread = threading.Thread(target=run_ros2_node, daemon=True)
    ros2_thread.start()
    
    timer_thread = threading.Thread(target=central_timer_worker, daemon=True)
    timer_thread.start()
    
    try:
        socketio.run(app, debug=False, host='0.0.0.0', port=5000)
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass