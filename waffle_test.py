import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import socket
import threading
import subprocess
import time

# ==========================================
# [설정] 게임컨트롤러 사용 여부 및 네트워크 설정
# ==========================================
USE_GAMECONTROLLER = True  # True로 설정 시 GameController와 연동
GC_PORT = 3838             # 로보컵 표준 포트

class SoccerBallTrackerGC(Node):
    def __init__(self):
        super().__init__('soccer_ball_tracker_gc')
        
        # 1. 모델 및 브릿지 로드
        self.model = YOLO('logi_total_best.pt') 
        self.bridge = CvBridge()
        
        # 2. 퍼블리셔 & 서브스크라이버
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # 제어 파라미터
        self.linear_speed = 0.15
        self.angular_speed = 0.5

        # --- GameController 관련 상태 변수 ---
        self.gc_state = 3 if not USE_GAMECONTROLLER else 0 # Default: Play(연동 안 할 때) or Initial
        self.ready_start_time = None
        self.ready_sub_phase = "FORWARD"  # FORWARD -> SEARCH -> ALIGN -> DONE
        
        if USE_GAMECONTROLLER:
            self.setup_network_and_gc()

    def setup_network_and_gc(self):
        """방화벽 해제 및 GameController 수신 스레드 시작"""
        self.get_logger().info("--- 네트워크 설정 및 GameController 연결 시작 ---")
        try:
            # 1. 방화벽 설정 (UDP 3838 포트 허용)
            subprocess.run(["sudo", "ufw", "allow", f"{GC_PORT}/udp"], check=False)
            
            # 2. 본인 IP 확인 (디버깅용)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            self.get_logger().info(f"로봇 IP 확인 완료: {local_ip}")

            # 3. GC 데이터 수신 스레드 실행
            self.gc_thread = threading.Thread(target=self.receive_gc_data, daemon=True)
            self.gc_thread.start()
        except Exception as e:
            self.get_logger().error(f"네트워크 설정 중 오류: {e}")

    def receive_gc_data(self):
        """GameController로부터 상태 데이터를 지속적으로 수신"""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', GC_PORT))
            
            while rclpy.ok():
                try:
                    data, _ = sock.recvfrom(1024)
                    if data[:4] == b'RGme': # RoboCup GameControlData Header
                        new_state = data[9] # 구조체 상의 state 인덱스
                        if self.gc_state != new_state:
                            self.get_logger().info(f"State 변경됨: {new_state}")
                            # READY 상태 진입 시 시퀀스 초기화
                            if new_state == 1:
                                self.ready_start_time = time.time()
                                self.ready_sub_phase = "FORWARD"
                            self.gc_state = new_state
                except:
                    pass

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape
        
        # YOLO 추론
        results = self.model(frame, verbose=False)
        
        twist = Twist()
        detected = False
        ball_center_x = None

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                ball_center_x = (x1 + x2) / 2
                detected = True
                break # 첫 번째 공만 타겟팅

        # ==========================================
        # [상태별 로직 분기]
        # ==========================================
        
        # 1. READY 상태 (값: 1)
        if self.gc_state == 1:
            self.handle_ready_logic(twist, detected, ball_center_x, width)

        # 2. SET 상태 (값: 2)
        elif self.gc_state == 2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("SET: 정지 상태", once=True)

        # 3. PLAY 상태 (값: 3)
        elif self.gc_state == 3:
            self.handle_play_logic(twist, detected, ball_center_x, width)
        
        # 아무 상태도 아닐 경우 (INITIAL 등) 정지
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

    def handle_ready_logic(self, twist, detected, ball_x, width):
        """READY 상태의 시퀀스 제어"""
        now = time.time()
        
        # A. 5초간 전진
        if self.ready_sub_phase == "FORWARD":
            if now - self.ready_start_time < 5.0:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            else:
                self.ready_sub_phase = "SEARCH"
        
        # B. 공 찾을 때까지 회전
        elif self.ready_sub_phase == "SEARCH":
            if not detected:
                twist.linear.x = 0.0
                twist.angular.z = 0.4 # 탐색 회전
            else:
                self.ready_sub_phase = "ALIGN"
        
        # C. 공이 인식되면 속도 줄여 중앙 정렬 후 정지
        elif self.ready_sub_phase == "ALIGN":
            if detected:
                error = ball_x - (width / 2)
                if abs(error) > 30: # 중앙 오차 범위 30픽셀
                    twist.linear.x = 0.0
                    twist.angular.z = 0.15 if error < 0 else -0.15 # 저속 회전
                else:
                    self.ready_sub_phase = "DONE"
            else:
                self.ready_sub_phase = "SEARCH" # 놓치면 다시 탐색
        
        # D. 정지
        elif self.ready_sub_phase == "DONE":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

    def handle_play_logic(self, twist, detected, ball_x, width):
        """기존의 3분할 추적 로직 (PLAY)"""
        if detected:
            if ball_x < width / 3:
                twist.linear.x = 0.05
                twist.angular.z = self.angular_speed
            elif ball_x > 2 * width / 3:
                twist.linear.x = 0.05
                twist.angular.z = -self.angular_speed
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

def main():
    rclpy.init()
    node = SoccerBallTrackerGC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
