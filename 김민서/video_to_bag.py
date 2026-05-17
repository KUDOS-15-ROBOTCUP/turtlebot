import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rosbag2_py
from rclpy.serialization import serialize_message
import os

def create_bag_from_video(video_path, bag_path):
    # 1. 중복 파일(폴더) 존재 여부 확인 (삭제 대신 중단하도록 변경)
    if os.path.exists(bag_path):
        print(f"에러: '{bag_path}' 폴더가 이미 존재합니다.")
        print("기존 데이터를 보호하기 위해 변환을 중단합니다. 스크립트 하단의 'bag_folder' 이름을 변경한 후 다시 실행해 주세요.")
        return  # 변환을 진행하지 않고 함수 종료

    print(f"✅ 알림: '{bag_path}' 폴더가 없습니다. 안전하게 새로 생성합니다.")

    # 2. 영상 파일 열기
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"에러: {video_path} 영상을 열 수 없습니다. 경로를 확인하세요.")
        return

    # 3. 전체 프레임 수 확인
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"🎥 영상 인식 완료! 총 변환할 프레임 수: {total_frames} 프레임")

    bridge = CvBridge()
    
    # 4. 로스백 작가(Writer) 설정 (Humble 호환)
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', 
        output_serialization_format='cdr'
    )
    writer.open(storage_options, converter_options)

    # 5. 토픽 설정
    topic_info = rosbag2_py.TopicMetadata(
        name='/camera/image_raw',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    )
    writer.create_topic(topic_info)

    print(f"🚀 본격적인 변환 시작: {video_path} -> {bag_path}")
    
    frame_count = 0
    # 약 30FPS 기준 (1프레임당 33,333,333 나노초)
    time_step_ns = 33333333 
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # OpenCV 이미지를 ROS Image 메시지로 변환
        msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # 타임스탬프 설정
        current_time_ns = frame_count * time_step_ns
        msg.header.stamp.sec = current_time_ns // 1000000000
        msg.header.stamp.nanosec = current_time_ns % 1000000000
        msg.header.frame