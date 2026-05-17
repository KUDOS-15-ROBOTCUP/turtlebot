import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import socket

class IntegratedGCBridge(Node):
    def __init__(self):
        super().__init__('integrated_gc_bridge')
        
        # 1. 발행자 설정: Gazebo의 터틀봇 제어
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 2. UDP 소켓 설정: GameController 신호 직접 수신
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', 3838))
        self.sock.setblocking(False)
        
        # 상태 변수 및 매핑
        self.current_state = "INITIAL"
        self.state_map = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAY", 4: "FINISH"}
        
        # 3. 타이머 설정: 0.1초마다 UDP 수신 및 로직 실행
        self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info("통합 GC 브릿지 노드가 시작되었습니다. 버튼을 기다리는 중...")

    def main_loop(self):
        try:
            # UDP 데이터 수신
            data, addr = self.sock.recvfrom(2048)
            if data and len(data) > 10:
                new_state = self.state_map.get(data[9], "UNKNOWN")
                
                # 상태가 변했을 때만 로그 출력 및 동작 실행
                if self.current_state != new_state:
                    self.get_logger().info(f"상태 변경 감지: {self.current_state} -> {new_state}")
                    self.current_state = new_state
                    self.execute_policy()
        except (BlockingIOError, socket.error):
            pass

    def execute_policy(self):
        move_cmd = Twist()

        if self.current_state == "READY":
            self.get_logger().info("READY: 전략 위치로 이동 (전진)")
            move_cmd.linear.x = 0.15
            
        elif self.current_state == "SET":
            self.get_logger().info("SET: 정지 및 대기")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            
        elif self.current_state == "PLAY":
            self.get_logger().info("PLAY: 경기 시작 (회전)")
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.5
            
        elif self.current_state in ["INITIAL", "FINISH"]:
            self.get_logger().info("종료 또는 대기: 정지")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0

        # Gazebo에 명령 전달
        self.cmd_vel_pub.publish(move_cmd)

def main():
    rclpy.init()
    node = IntegratedGCBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
