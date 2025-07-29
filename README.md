# ADAS System (Advanced Driver Assistance System)

Hệ thống hỗ trợ lái xe tiên tiến sử dụng Raspberry Pi, ESP32 và STM32 để cung cấp các tính năng an toàn lái xe thông minh.

## 🚗 Tổng quan

ADAS System là một dự án phát triển hệ thống hỗ trợ lái xe tiên tiến bao gồm:
- **Phát hiện làn đường** (Lane Detection)
- **Phát hiện vật cản** (Obstacle Detection) 
- **Cảnh báo điểm mù** (Blind Spot Detection)
- **Giám sát tốc độ động cơ**
- **Giao diện người dùng** với màn hình TFT và LVGL
- **Kết nối Bluetooth** cho điều khiển từ xa

## 🏗️ Kiến trúc hệ thống

### 1. **ESP32 Main ECU** 
- Vi điều khiển trung tâm
- **FreeRTOS** multitasking cho xử lý song song
- Giao diện TFT với LVGL UI
- Kết nối Bluetooth BLE
- CAN Bus communication
- **Hệ thống phanh khẩn cấp** (Emergency Braking System)
- Buzzer cảnh báo âm thanh
- **Real-time processing** cho các tình huống nguy hiểm

### 2. **ROS2 ADAS Package**
- **Lane Detection Node**: Phát hiện làn đường sử dụng OpenCV
- **Obstacle Detection Node**: Phát hiện vật cản qua LiDAR
- **CAN Communication**: Gửi dữ liệu qua CAN bus

### 3. **STM32 Modules**
- **Blind Spot Detection**: STM32F103C8T6 cho cảnh báo điểm mù
- **Motor Encoder**: STM32F103C8T6 cho giám sát động cơ

## 📁 Cấu trúc thư mục

```
├── ESP32_Main_ECU.ino          # Code ESP32 chính
├── ADAS_system/                # ROS2 package
│   ├── ADAS_system/
│   │   ├── lane_detection.py   # Node phát hiện làn đường
│   │   ├── obstacle_detect.py  # Node phát hiện vật cản
│   │   └── CAN_send.py        # CAN communication
│   └── launch/
│       └── adas_system_launch.py
├── Blind_spot/                # STM32 blind spot detection
└── Motor_encoder/             # STM32 motor encoder monitoring
```

## 🛠️ Yêu cầu hệ thống

### Phần cứng
- **ESP32** với màn hình TFT
- **STM32F103C8T6** (x2)
- **LiDAR sensor** (cho nhận diện vật cản)
- **Camera** (cho nhận diện làn đường)
- **IR Sensor (x2)** (cho cảnh báo điểm mù)
- **CAN transceiver modules**
- **Buzzer** cảnh báo

### Phần mềm
- **ROS2** (Humble)
- **Arduino IDE** (sẽ chuyển sang ESP-IDF)
- **STM32CubeIDE**
- **Python 3.8+**

## 📦 Dependencies

### ROS2 Package
```bash
# Python packages
- rclpy
- sensor_msgs
- std_msgs
- cv_bridge
- opencv-python
- numpy
- python3-can
```

### ESP32 Libraries
```cpp
- TFT_eSPI
- LVGL
- BLEDevice
- FreeRTOS
```

## 🚀 Cài đặt và chạy

### 1. Setup ROS2 Package
```bash
cd ~/ros2_ws/src
git clone https://github.com/phatdoan301/ADAS-System.git
cd ~/ros2_ws
colcon build --packages-select ADAS_system
source install/setup.bash
```

### 2. Chạy ADAS System
```bash
# Launch toàn bộ system
ros2 launch ADAS_system adas_system_launch.py

# Hoặc chạy từng node riêng lẻ
ros2 run ADAS_system lane_detection
ros2 run ADAS_system obstacle_detect
```

### 3. Flash ESP32
- Mở `ESP32_Main_ECU.ino` trong Arduino IDE
- Cấu hình board ESP32 và upload

### 4. Flash STM32
- Mở project trong STM32CubeIDE
- Build và flash cho cả Blind_spot và Motor_encoder

## 📊 Topics ROS2

| Topic | Message Type | Mô tả |
|-------|-------------|--------|
| `/image_raw` | sensor_msgs/Image | Camera input |
| `/lane_detected` | sensor_msgs/Image | Lane detection output |
| `/lane_status` | std_msgs/String | Lane status info |
| `/scan` | sensor_msgs/LaserScan | LiDAR data |
| `/obstacle_dist` | std_msgs/Int16 | Obstacle distance |


## ⚙️ Cấu hình

### CAN Bus
- **ESP32 CAN TX**: GPIO 4
- **ESP32 CAN RX**: GPIO 5
- **Baud rate**: 500 kbps

### FreeRTOS Tasks
- **UI Task**: Cập nhật giao diện LVGL (Priority: 1)
- **CAN Task**: Xử lý CAN communication (Priority: 3)
- **Emergency Task**: Xử lý phanh khẩn cấp (Priority: 5 - Highest)
- **Bluetooth Task**: Xử lý BLE communication (Priority: 2)

### Emergency Braking System
- **Trigger distance**: < 0.5m
- **Response time**: < 100ms
- **Brake signal**: Gửi tín hiệu phanh khẩn cấp tới khối điều khiển động cơ
- **Emergency override**: Tự động can thiệp khi cần thiết
