import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoboCupGCNode(Node):
    def __init__(self):
        super().__init__('robocup_gc_node')
        
        # --- 설정부 ---
        self.declare_parameter('team_number', 1)  # 본인의 팀 번호로 수정하세요
        self.team_num = self.get_parameter('team_number').value
        self.target_topic = '/cmd_vel'
        
        # ROS2 퍼블리셔
        self.publisher = self.create_publisher(Twist, self.target_topic, 10)
        
        # UDP 소켓 설정
        self.gc_port = 3838  # GAMECONTROLLER_DATA_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.gc_port))
        self.sock.setblocking(False)

        # 게임 상태 상수 정의 (헤더 참조)
        self.STATE_INITIAL = 0
        self.STATE_READY = 1
        self.STATE_SET = 2
        self.STATE_PLAYING = 3
        self.STATE_FINISHED = 4

        # 타이머 (10Hz)
        self.create_timer(0.1, self.main_loop)
        self.get_logger().info(f"GC Listener started for Team #{self.team_num}")

    def main_loop(self):
        try:
            # 패킷 수신
            data, _ = self.sock.recvfrom(2048) # 충분한 버퍼 크기
            
            # 1. 헤더 체크 ("RGme")
            if data[:4] != b'RGme':
                return

            # 2. 메인 데이터 파싱
            # 포맷: 4s(header), H(version), B(packetNum), B(players), B(gameType), B(state), ...
            # 헤더 파일 구조에 따라 state는 10번째 바이트(index 9)에 위치함
            header_part = struct.unpack('4s H B B B B B B B 4s B H H H', data[:24])
            
            game_state = header_part[5]      # state
            secondary_state = header_part[8] # secondaryState
            
            # 3. 로봇 제어 로직 호출
            self.control_turtlebot(game_state)

        except BlockingIOError:
            # 수신된 패킷이 없을 경우 무시
            pass
        except Exception as e:
            self.get_logger().error(f"Error parsing GC data: {e}")

    def control_turtlebot(self, state):
        msg = Twist()

        if state == self.STATE_INITIAL:
            # 초기 상태: 정지
            msg.linear.x = 0.0
            self.get_logger().info("STATE: INITIAL - Standing by", throttle_duration_sec=2.0)

        elif state == self.STATE_READY:
            # READY: 전략적 위치로 이동 (가제보 테스트용 전진)
            msg.linear.x = 0.1
            self.get_logger().info("STATE: READY - Moving to kickoff position", throttle_duration_sec=2.0)

        elif state == self.STATE_SET:
            # SET: 모든 이동 정지 (킥오프 대기)
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("STATE: SET - Ready to play", throttle_duration_sec=2.0)

        elif state == self.STATE_PLAYING:
            # PLAYING: 자율 주행 모드 활성화 (예: 전진 및 회전)
            msg.linear.x = 0.2
            msg.angular.z = 0.1
            self.get_logger().info("STATE: PLAYING - Executing match logic", throttle_duration_sec=2.0)

        elif state == self.STATE_FINISHED:
            # FINISHED: 경기 종료 및 정지
            msg.linear.x = 0.0
            self.get_logger().info("STATE: FINISHED - Match over", throttle_duration_sec=2.0)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoboCupGCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
