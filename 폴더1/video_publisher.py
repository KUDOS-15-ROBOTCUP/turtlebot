import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # 30fps 영상 기준 (1초에 30번 콜백 실행)
        timer_period = 1.0 / 30.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 홈 폴더에 있는 영상 경로 
        self.cap = cv2.VideoCapture('/home/minseo/robocup_recording_1778065491.mp4') 
        self.bridge = CvBridge()
        self.get_logger().info('비디오 퍼블리셔 노드가 시작되었습니다!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 이미지를 ROS Image 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 메시지에 현재 시간(타임스탬프)과 프레임 ID 추가
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'
            
            self.publisher_.publish(msg)
        else:
            self.get_logger().info('영상이 끝났습니다. 처음부터 다시 재생합니다.')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
