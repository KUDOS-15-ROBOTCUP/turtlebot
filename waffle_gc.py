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
USE_GAMECONTROLLER = True  
GC_PORT = 3838             

class SoccerBallTrackerGC(Node):
    def __init__(self):
        super().__init__('soccer_ball_tracker_gc')
        
        self.get_logger().info("YOLO 모델 로딩 중...")
        self.model = YOLO('logi_total_best.pt') 
        self.bridge = CvBridge()
        
        # ROS2 통신 설정
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bag_subscription = self.create_subscription(
            Twist, '/cmd_vel_bag', self.bag_callback, 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
            
        self.bag_twist = Twist() # 로스백 데이터 저장용
        
        # GC 상태 변수
        self.gc_state = 0 
        self.state_map = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAY", 4: "FINISHED"}
        
        self.ready_start_time = None
        self.ready_sub_phase = "FORWARD"
        
        # 비전 처리 결과 저장용 변수 (새로 추가)
        self.detected = False
        self.ball_center_x = 0.0
        self.img_width = 640
        
        # 🌟 핵심: 1초에 10번 무조건 실행되는 제어 전용 타이머
        self.create_timer(0.1, self.control_loop)
        
        if USE_GAMECONTROLLER:
            self.setup_network_and_gc()

    def setup_network_and_gc(self):
        self.get_logger().info("--- 외부 네트워크 GameController 연결 대기 ---")
        try:
            subprocess.run(["sudo", "ufw", "allow", f"{GC_PORT}/udp"], check=False)
            self.gc_thread = threading.Thread(target=self.receive_gc_data, daemon=True)
            self.gc_thread.start()
        except Exception as e:
            self.get_logger().error(f"네트워크 설정 중 오류: {e}")

    def receive_gc_data(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind(('0.0.0.0', GC_PORT))
            
            while rclpy.ok():
                try:
                    data, addr = sock.recvfrom(1024)
                    if len(data) >= 10 and data[:4] == b'RGme': 
                        new_state = data[9]
                        
                        if self.gc_state != new_state:
                            state_name = self.state_map.get(new_state, "UNKNOWN")
                            self.get_logger().info(f"🚨 심판 지시: {state_name} ({new_state})")
                            
                            if new_state == 1:
                                self.ready_start_time = time.time()
                                self.ready_sub_phase = "FORWARD"
                                
                            self.gc_state = new_state
                except Exception:
                    pass

    def bag_callback(self, msg):
        """로스백에서 들어오는 데이터만 업데이트"""
        self.bag_twist = msg

    def image_callback(self, msg):
        """비전(YOLO) 처리는 영상이 들어올 때만 독립적으로 수행"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.img_width = frame.shape[1]
        
        results = self.model(frame, verbose=False)
        self.detected = False

        # 🌟 1. YOLO가 공을 인식한 네모 박스가 그려진 이미지 생성
        annotated_frame = results[0].plot()

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                self.ball_center_x = (x1 + x2) / 2
                self.detected = True
                break
                
        # 🌟 2. 화면에 실시간 영상 띄우기 (OpenCV 창)
        cv2.imshow("Turtlebot Camera (YOLO View)", annotated_frame)
        cv2.waitKey(1)

    def control_loop(self):
        """🌟 카메라와 독립적으로 10Hz마다 로봇의 속도를 결정하고 전송하는 심장부"""
        twist = Twist()

        # 1. PLAY (경기 중) - 로스백 속도 덮어쓰기
        # 1. PLAY (경기 중) - 공 위치에 따른 추적 또는 로스백
        if self.gc_state == 3:
            self.handle_play_logic(twist)
            
            # 로그 출력 (디버깅용)
            self.get_logger().info(f"▶ PLAY 중 (v={twist.linear.x:.2f}, w={twist.angular.z:.2f})", throttle_duration_sec=1.0)
            
        # 2. READY (경기 준비) - 위치 잡기 시퀀스
        elif self.gc_state == 1:
            self.handle_ready_logic(twist)
            
        # 3. SET, INITIAL 등 - 정지
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self.gc_state == 2:
                self.get_logger().info("⏸ SET: 완전 정지 대기 중", throttle_duration_sec=2.0)

        self.publisher.publish(twist)

    def handle_ready_logic(self, twist):
        now = time.time()
        
        if self.ready_sub_phase == "FORWARD":
            if now - self.ready_start_time < 5.0:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            else:
                self.ready_sub_phase = "SEARCH"
        
        elif self.ready_sub_phase == "SEARCH":
            if not self.detected:
                twist.linear.x = 0.0
                twist.angular.z = 0.4 
            else:
                self.ready_sub_phase = "ALIGN"
        
        elif self.ready_sub_phase == "ALIGN":
            if self.detected:
                error = self.ball_center_x - (self.img_width / 2)
                if abs(error) > 30: 
                    twist.linear.x = 0.0
                    twist.angular.z = 0.15 if error < 0 else -0.15
                else:
                    self.ready_sub_phase = "DONE"
            else:
                self.ready_sub_phase = "SEARCH"
        
        elif self.ready_sub_phase == "DONE":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

    def handle_play_logic(self, twist):
        if self.detected:
            # 화면을 3등분하여 공의 위치에 따라 다르게 움직임
            if self.ball_center_x < self.img_width / 3:
                # [왼쪽] 공이 왼쪽에 있음 -> 좌회전 하면서 직진 (커브)
                twist.linear.x = 0.05
                twist.angular.z = 0.3
            elif self.ball_center_x > 2 * self.img_width / 3:
                # [오른쪽] 공이 오른쪽에 있음 -> 우회전 하면서 직진 (커브)
                twist.linear.x = 0.05
                twist.angular.z = -0.3
            else:
                # [가운데] 공이 중앙에 있음 -> 강하게 직진!
                twist.linear.x = 0.15
                twist.angular.z = 0.0
        else:
            # [공이 안 보일 때] -> 이때만 rosbag 데이터를 따름
            twist.linear.x = self.bag_twist.linear.x
            twist.angular.z = self.bag_twist.angular.z

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