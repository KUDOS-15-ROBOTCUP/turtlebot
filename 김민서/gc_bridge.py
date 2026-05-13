import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket
import struct

class GCBridge(Node):
    def __init__(self):
        super().__init__('gc_bridge')
        
        # 1. 발행자 설정: 터틀봇 제어용
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 2. UDP 소켓 설정 (GameController 포트 3838)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', 3838))
        self.sock.setblocking(False)
        
        # 3. 상태 매핑 (RoboCupGameControlData.h 정의 기준)
        self.state_map = {
            0: "INITIAL",  # STATE_INITIAL
            1: "READY",    # STATE_READY
            2: "SET",      # STATE_SET
            3: "PLAY",     # STATE_PLAYING
            4: "FINISH"    # STATE_FINISHED
        }
        
        self.current_state = "INITIAL"
        self.get_logger().info("로보컵 통합 브릿지(GC + Control) 가동 시작!")
        
        # 4. 주기적 실행 타이머 (10Hz)
        self.create_timer(0.1, self.main_loop)

    def main_loop(self):
        # UDP 데이터 수신 시도
        try:
            data, addr = self.sock.recvfrom(2048)
            if data and len(data) >= 10:
                # 헤더 "RGme" 확인 및 10번째 바이트(index 9) 추출
                if data[:4] == b'RGme':
                    state_num = data[9]
                    new_state = self.state_map.get(state_num, "UNKNOWN")
                    
                    if self.current_state != new_state:
                        self.get_logger().info(f"심판 신호 감지: {self.current_state} -> {new_state}")
                        self.current_state = new_state
        except (BlockingIOError, socket.error):
            pass

        # 현재 상태에 따른 이동 정책 실행
        self.execute_policy()

    def execute_policy(self):
        move_cmd = Twist()

        # 상태별 동작 로직 (들여쓰기 주의!)
        if self.current_state == "READY":
            # 전략 위치로 이동
            move_cmd.linear.x = 0.15 
            
        elif self.current_state == "SET":
            # 완전 정지 및 대기
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            
        elif self.current_state == "PLAY":
            # 경기 시작: 전진하며 회전 탐색
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.5
            
        elif self.current_state in ["FINISH", "INITIAL"]:
            # 정지 상태
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GCBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()