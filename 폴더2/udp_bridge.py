import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class UDPReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')
        self.publisher_ = self.create_publisher(String, 'game_status', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', 3838))
        self.sock.setblocking(False)
        
        # 상태 숫자와 문자열 매핑
        self.state_map = {
            0: "INITIAL",
            1: "READY",
            2: "SET",
            3: "PLAY",
            4: "FINISH"
        }
        
        self.get_logger().info("실시간 상태 분석 브릿지 가동 시작!")
        self.create_timer(0.1, self.receive_callback)

    def receive_callback(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            if data and len(data) > 10:
                # 패킷의 10번째 바이트(index 9)가 현재 경기 상태를 나타냅니다.
                state_num = data[9] 
                state_str = self.state_map.get(state_num, "UNKNOWN")
                
                msg = String()
                msg.data = state_str
                self.publisher_.publish(msg)
                
                # 상태가 바뀔 때만 로그를 찍고 싶다면 아래처럼 활용
                # self.get_logger().info(f"현재 심판 신호: {state_str}")
        except (BlockingIOError, socket.error):
            pass

def main():
    rclpy.init()
    node = UDPReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
