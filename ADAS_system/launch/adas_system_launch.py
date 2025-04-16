from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node LiDAR từ package sllidar_ros2
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters = [
                {'channel_type': 'serial'},
                {'serial_port': '/dev/ttyUSB0'},  # Cổng serial của LiDAR
                {'serial_baudrate': 115200},
                {'frame_id': 'laser'},
            ]
        ),
        # Node camera từ package usb_cam
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[
                {'video_device': '/dev/video0'},  # Thiết bị camera
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'camera'},
                {'io_method': 'mmap'},
                {'camera_name': 'usb_cam'},
                {'image_topic': '/image_raw'}  # Topic xuất bản hình ảnh
            ]
        ),

        # Node đo khoảng cách
        Node(
            package='ADAS_system',
            executable='obstacle_detect',
            name='distance_node',
            output='screen',
            parameters=[
                {'topic_input': '/scan'},  # Nhận dữ liệu từ LiDAR
                {'max_distance': 0.5},
                {'topic_output': '/obstacle_dist'}
            ]
        ),

        # Node nhận diện làn đường
        Node(
            package='ADAS_system',
            executable='lane_detection',
            name='lane_detection_node',
            output='screen',
            parameters=[
                {'input_image_topic': '/image_raw'},  # Nhận hình ảnh từ camera
                {'output_lane_topic': '/lane_status'}
            ]
        ),

        # Node CAN (trong robot_control_pkg)
        Node(
            package='ADAS_system',
            executable='CAN_send',
            name='can_node',
            output='screen',
            parameters=[
                {'can_interface': 'can0'},  # Giao diện CAN
                {'baudrate': 500000}
            ]
        ),
    ])
