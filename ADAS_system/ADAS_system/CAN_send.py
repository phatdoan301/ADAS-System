import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String
import can


class CanPublisherNode(Node):
    def __init__(self):
        super().__init__('can_publisher_node')
        # Subcribe topic /obstacle_dist
        self.obstacle_sub = self.create_subscription(
            Int16,
            '/obstacle_dist',
            self.obstacle_callback,
            10)
        # Subcribe topic /lane_status
        self.lane_sub = self.create_subscription(
            String,
            '/lane_status',
            self.lane_callback,
            10)
        try:
            self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
            self.get_logger().info('CAN bus initialized on can0')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {str(e)}')
            self.can_bus = None

        self.obstacle_distance = 0  # Mặc định
        self.lane_status = "Giua lan"  # Mặc định
        self.data_changed = False  # Đánh dấu dữ liệu có thay đổi

        self.timer = self.create_timer(0.1, self.send_can_message)

    def obstacle_callback(self, msg):
        if self.obstacle_distance != msg.data:  # Chỉ cập nhật nếu dữ liệu thay đổi
            self.obstacle_distance = msg.data
            self.data_changed = True
            self.get_logger().info(f'Updated obstacle distance: {self.obstacle_distance} cm')
    
    def lane_callback(self, msg):
        if self.lane_status != msg.data:  # Chỉ cập nhật nếu dữ liệu thay đổi
            self.lane_status = msg.data
            self.data_changed = True
            self.get_logger().info(f'Updated lane status: {self.lane_status}')

    def send_can_message(self):
        if self.can_bus is None or not self.data_changed:
            return  # Không gửi nếu không có dữ liệu mới

        # Mã hóa trạng thái lệch làn thành số nguyên
        lane_code = 0  # Giữa làn
        if self.lane_status == 'Lech trai':
            lane_code = 1
        elif self.lane_status == 'Lech phai':
            lane_code = 2
            
        distance = self.obstacle_distance
        
        data = bytearray([lane_code, distance, 0])

        # Tạo thông điệp CAN
        msg = can.Message(
            arbitration_id=0x02,  # CAN ID
            data=data,
            is_extended_id=False
        )

        try:
            self.can_bus.send(msg)
            self.get_logger().info(f'Sent CAN message: ID=0x02, Lane={lane_code}, Distance={self.obstacle_distance} cm')
            self.data_changed = False  # Đặt lại trạng thái sau khi gửi
        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {str(e)}')

    def destroy_node(self):
        if self.can_bus:
            self.can_bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    CAN_node = CanPublisherNode()
    try:
        rclpy.spin(CAN_node)
    except KeyboardInterrupt:
        pass
    finally:
        CAN_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
