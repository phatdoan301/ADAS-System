import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectNode(Node):
    def __init__(self):
        super().__init__('lane_detect_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Thay bằng topic của camera
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/lane_detected', 10)
        self.lane_status_publisher = self.create_publisher(String, '/lane_status', 10)
        self.roi_publisher = self.create_publisher(Image, '/ROI', 10)

    def preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 80, 220)
        return edges

    def define_roi(self, edges, width, height):
        mask = np.zeros_like(edges)
        roi_corners = np.array([[
            (0, height * 0.5),
            (0, height),
            (width, height),
            (width, height * 0.5),
            (width * 0.7, height * 0.2),
            (width * 0.25, height * 0.2)
        ]], dtype=np.int32)
        cv2.fillPoly(mask, roi_corners, 255)
        roi = cv2.bitwise_and(edges, mask)
        return roi

    def detect_lane(self, image, roi, width, height):
        lines = cv2.HoughLinesP(roi, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=150)
        
        line_image = np.zeros_like(image)
        left_lane = []
        right_lane = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 != 0:  # Tránh chia cho 0
                    slope = (y2 - y1) / (x2 - x1)
                    if slope < -0.5:  # Làn trái
                        left_lane.append((x1, y1, x2, y2))
                    elif slope > 0.5:  # Làn phải
                        right_lane.append((x1, y1, x2, y2))
                    if abs(slope) > 0.5:  # Lọc đường ngang (độ dốc nhỏ)
                        cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

        left_x = np.mean([line[0] for line in left_lane] + [line[2] for line in left_lane]) if left_lane else 0
        right_x = np.mean([line[0] for line in right_lane] + [line[2] for line in right_lane]) if right_lane else width
        lane_center = (left_x + right_x) / 2
        car_position = width // 2

        # Kiểm tra lệch làn
        deviation = car_position - lane_center
        #print(f"vi tri xe: {car_position}, trung tam: {lane_center}, khoang cach: {deviation}")
        status = "Giua lan"
        if deviation > 100:
            status = "Lech phai"
        elif deviation < -100:
            status = "Lech trai"
        
        result = cv2.addWeighted(image, 0.8, line_image, 1, 0)

        left_distance = abs(lane_center - left_x) if left_lane else 0
        right_distance = abs(right_x - lane_center) if right_lane else 0

        # Vẽ khoảng cách
        y_position = height - 50  # Vị trí y để vẽ đường/mũi tên
        if left_lane:
            # Vẽ đường từ làn trái tới tâm
            cv2.line(result, (int(left_x), y_position), (int(lane_center), y_position), (255, 255, 0), 2)
            # Ghi khoảng cách
            cv2.putText(result, f"{int(left_distance)} px", (int(left_x + (lane_center - left_x) / 2), y_position - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        if right_lane:
            # Vẽ đường từ làn phải tới tâm
            cv2.line(result, (int(lane_center), y_position), (int(right_x), y_position), (255, 255, 0), 2)
            # Ghi khoảng cách
            cv2.putText(result, f"{int(right_distance)} px", (int(lane_center + (right_x - lane_center) / 2), y_position - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        

        cv2.putText(result, status, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.line(result, (int(lane_center), height), (int(lane_center), height // 2), (255, 0, 0), 2)
        cv2.line(result, (car_position, height), (car_position, height // 2), (0, 0, 255), 2)
        return result, status

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        edges = self.preprocess_image(cv_image)
        height, width = edges.shape
        roi = self.define_roi(edges, width, height)
        final_output, status_msg = self.detect_lane(cv_image, roi, width, height)
        output_msg = self.bridge.cv2_to_imgmsg(final_output, "bgr8")
        output_roi = self.bridge.cv2_to_imgmsg(roi, "mono8")
        self.roi_publisher.publish(output_roi)
        self.publisher.publish(output_msg)
        lane_lean_msg = String()
        lane_lean_msg.data = status_msg
        self.lane_status_publisher.publish(lane_lean_msg)
        #self.get_logger().info(status_msg)


def main(args=None):
    rclpy.init(args=args)
    LaneDetect = LaneDetectNode()
    try:
        rclpy.spin(LaneDetect)
    except KeyboardInterrupt:
        pass
    finally:
        LaneDetect.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
