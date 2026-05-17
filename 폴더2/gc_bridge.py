import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String # GC 상태를 문자열로 받는다고 가정

class Turtlebot3RefereePolicy(Node):
    def __init__(self):
        super().__init__('tb3_referee_policy')
        
        # 1. 발행자: 터틀봇 움직임 제어
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 2. 구독자: GC 브릿지 노드로부터 상태 수신
        self.create_subscription(String, 'game_status', self.status_callback, 10)
        
        self.current_state = "INITIAL"
        self.get_logger().info("터틀봇 로보컵 예습 노드가 시작되었습니다.")

    def status_callback(self, msg):
        new_state = msg.data
        if self.current_state != new_state:
            self.get_logger().info(f"상태 변경: {self.current_state} -> {new_state}")
            self.current_state = new_state
            self.execute_policy()

    def execute_policy(self):
        move_cmd = Twist()

        if self.current_state == "READY":
            # 예: 자기 진영으로 이동 (여기선 단순 전진으로 예시)
            self.get_logger().info("READY: 전략 위치로 이동합니다.")
            move_cmd.linear.x = 0.15 
            
        elif self.current_state == "SET":
            # 완전 정지
            self.get_logger().info("SET: 모든 동작을 멈추고 대기합니다.")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            
        elif self.current_state == "PLAY":
            # 경기 시작: 공 찾기 또는 공격 로직 실행
            self.get_logger().info("PLAY: 경기를 시작합니다!")
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.5 # 제자리 회전하며 공 찾는 시늉
            
        elif self.current_state == "FINISH" or self.current_state == "INITIAL":
            # 정지
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3RefereePolicy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
    # 반드시 이 두 줄이 있어야 프로그램이 시작됩니다!
if __name__ == '__main__':
    main()
