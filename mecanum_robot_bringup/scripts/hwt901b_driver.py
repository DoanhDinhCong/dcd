#!/usr/bin/env python3
"""
hwt901b_driver.py - Driver IMU HWT901B cho ROS2
================================================
Driver đọc dữ liệu từ cảm biến IMU HWT901B (9 trục) qua Serial và publish lên ROS2.

THÔNG SỐ KỸ THUẬT HWT901B:
--------------------------
- Cảm biến: MPU9250 (6 trục) + AK8963 (3 trục từ kế)
- Đầu ra: Gia tốc, Con quay hồi chuyển, Góc Euler, Quaternion, Từ trường
- Giao tiếp: UART (Serial), I2C, hoặc USB
- Tốc độ truyền mặc định: 9600 baud (có thể cấu hình lên 115200)
- Tần số xuất dữ liệu: 0.2Hz - 200Hz (mặc định: 10Hz)
- Phạm vi:
  - Gia tốc kế: ±16g
  - Con quay hồi chuyển: ±2000°/giây
  - Góc: ±180° (Roll/Pitch), 0-360° (Yaw)

GIAO THỨC TRUYỀN DỮ LIỆU:
-------------------------
Mỗi gói tin (packet) 11 bytes:
┌────────┬────────┬──────────────┬──────────┐
│  0x55  │  Loại  │  Dữ liệu(8B) │ Checksum │
│ Header │  1B    │              │   1B     │
└────────┴────────┴──────────────┴──────────┘

Mã loại (Type codes):
- 0x50: Thời gian
- 0x51: Gia tốc (ax, ay, az)
- 0x52: Vận tốc góc (wx, wy, wz)
- 0x53: Góc Euler (roll, pitch, yaw)
- 0x54: Từ trường (mx, my, mz)
- 0x55: Trạng thái cổng
- 0x56: Áp suất không khí + độ cao
- 0x57: Kinh độ + vĩ độ
- 0x58: Vận tốc mặt đất
- 0x59: Quaternion (q0, q1, q2, q3)

DRIVER NÀY CHỈ SỬ DỤNG:
- 0x51: Gia tốc
- 0x52: Vận tốc góc
- 0x53: Góc Euler

LƯU Ý QUAN TRỌNG:
- ✅ Tốc độ truyền mặc định 9600 baud (nếu chưa cấu hình IMU)
- ⚠️ Cần cấu hình IMU lên 115200 nếu muốn tốc độ cao hơn
- ⚠️ Cổng thường là /dev/ttyUSB2 hoặc ttyUSB1


"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import math
import time


class HWT901BDriver(Node):
    """
    Driver cho cảm biến IMU HWT901B
    
    Chức năng:
    ----------
    1. Kết nối với IMU qua cổng Serial
    2. Đọc và phân tích 3 loại gói tin:
       - 0x51: Gia tốc (gia tốc tuyến tính)
       - 0x52: Con quay hồi chuyển (vận tốc góc)
       - 0x53: Góc (góc Euler: roll, pitch, yaw)
    3. Chuyển đổi dữ liệu:
       - Raw int16 → đơn vị vật lý (m/s², rad/s, rad)
       - Góc Euler → Quaternion
    4. Xuất bản message sensor_msgs/Imu
    
    Quy trình hoạt động:
    -------------------
    Serial → read_imu_data() → phân tích gói tin → 
    tích lũy 3 loại → publish_imu() → topic /imu/data
    
    HỆ TỌA ĐỘ:
    ----------
    HWT901B sử dụng hệ tọa độ ENU (Đông-Bắc-Trên):
    - X: Hướng đông (phía trước IMU khi nằm phẳng)
    - Y: Hướng bắc (bên trái)
    - Z: Hướng lên (vuông góc bề mặt)
    
    ROS2 cũng dùng ENU nên không cần chuyển đổi trục.
    """
    
    def __init__(self):
        super().__init__('hwt901b_driver')
        
        # =====================================================================
        # THAM SỐ CẤU HÌNH
        # =====================================================================
        
        # Cổng Serial - Cổng kết nối IMU
        # Kiểm tra: ls -l /dev/ttyUSB*
        # Thường là: /dev/ttyUSB0, /dev/ttyUSB1, hoặc /dev/ttyUSB2
        self.declare_parameter('port', '/dev/ttyUSB2')
        
        # =====================================================================
        # TỐC ĐỘ TRUYỀN (BAUDRATE)
        # =====================================================================
        # ✅ ĐÃ SỬA: Vấn đề nghiêm trọng #2
        # 
        # HWT901B xuất xưởng mặc định: 9600 baud
        # Để sử dụng 115200 baud cần:
        #   1. Cài phần mềm WitMotion trên Windows
        #   2. Kết nối IMU qua USB
        #   3. Vào Settings → Baudrate → 115200
        #   4. Lưu cấu hình
        # 
        # Nếu CHƯA cấu hình → phải dùng 9600!
        # =====================================================================
        self.declare_parameter('baudrate', 9600)  # ✅ Giữ 9600 (mặc định xuất xưởng)
        
        # Frame ID - Khung tọa độ của IMU trong cây TF
        # Phải khớp với URDF: <link name="imu_link">
        self.declare_parameter('frame_id', 'imu_link')
        
        # Đọc giá trị tham số
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.get_logger().info('═' * 60)
        self.get_logger().info('Cấu hình Driver IMU HWT901B:')
        self.get_logger().info(f'  Cổng: {port}')
        self.get_logger().info(f'  Tốc độ truyền: {baudrate} baud')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info('═' * 60)
        
        # =====================================================================
        # BỘ ĐỆM DỮ LIỆU
        # =====================================================================
        # HWT901B gửi 3 loại gói tin riêng biệt, không đồng thời
        # Cần lưu trữ cho đến khi có đủ cả 3 loại
        
        # Bộ đệm gia tốc (m/s²) - Gia tốc tuyến tính
        # [ax, ay, az] - Gia tốc theo 3 trục X, Y, Z
        self.acc = [0.0, 0.0, 0.0]
        
        # Bộ đệm con quay (rad/s) - Vận tốc góc
        # [wx, wy, wz] - Vận tốc góc quanh 3 trục X, Y, Z
        self.gyro = [0.0, 0.0, 0.0]
        
        # Bộ đệm góc (rad) - Góc Euler
        # [roll, pitch, yaw] - Góc nghiêng, góc tầng, góc hướng
        self.angle = [0.0, 0.0, 0.0]
        
        # =====================================================================
        # CỜ TRẠNG THÁI
        # =====================================================================
        # ✅ ĐÃ THÊM: Theo dõi timeout (Vấn đề nhỏ #10)
        # 
        # Các cờ đánh dấu đã nhận loại dữ liệu nào
        # Chỉ xuất bản khi có đủ cả 3 loại (acc + gyro + angle)
        self.has_acc = False      # Đã có dữ liệu gia tốc?
        self.has_gyro = False     # Đã có dữ liệu con quay?
        self.has_angle = False    # Đã có dữ liệu góc?
        
        # Dấu thời gian của gói tin cuối cùng
        # Dùng để phát hiện timeout (IMU ngưng gửi dữ liệu)
        self.last_packet_time = time.time()
        
        # =====================================================================
        # KẾT NỐI SERIAL
        # =====================================================================
        try:
            # Khởi tạo cổng Serial
            # timeout=0.01: Đọc không chặn (non-blocking), chờ tối đa 10ms
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.01,       # Timeout 10ms
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Xóa bộ đệm cũ
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.get_logger().info(f'✅ Đã kết nối với HWT901B trên {port} @ {baudrate} baud')
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Không thể mở cổng serial {port}')
            self.get_logger().error(f'   Lỗi: {e}')
            self.get_logger().error(f'   Cách khắc phục:')
            self.get_logger().error(f'   1. Kiểm tra kết nối: ls -l /dev/ttyUSB*')
            self.get_logger().error(f'   2. Kiểm tra quyền: sudo chmod 666 {port}')
            self.get_logger().error(f'   3. Thêm user vào nhóm dialout: sudo usermod -aG dialout $USER')
            self.get_logger().error(f'   4. Kiểm tra nguồn điện IMU (5V)')
            raise
            
        except Exception as e:
            self.get_logger().error(f'❌ Lỗi không mong đợi: {e}')
            raise
        
        # =====================================================================
        # PUBLISHER ROS2
        # =====================================================================
        # Xuất bản dữ liệu IMU lên topic /imu/data
        # Loại message: sensor_msgs/Imu
        # QoS: 10 (kích thước hàng đợi)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # =====================================================================
        # TIMER - Đọc dữ liệu định kỳ
        # =====================================================================
        # Timer gọi read_imu_data() mỗi 0.01s (100Hz)
        # HWT901B xuất dữ liệu 10Hz, đọc 100Hz để không bỏ sót gói tin
        self.create_timer(0.01, self.read_imu_data)
        
        # Thống kê
        self.packet_count = 0      # Tổng số gói tin nhận được
        self.publish_count = 0     # Tổng số lần xuất bản
        self.error_count = 0       # Số lỗi checksum/phân tích
        
        self.get_logger().info('✅ Driver HWT901B đã khởi động!')
        self.get_logger().info('   Đang chờ dữ liệu từ IMU...')
        
    # =========================================================================
    # HÀM ĐỌC CHÍNH
    # =========================================================================
    
    def read_imu_data(self):
        """
        Đọc và phân tích dữ liệu từ IMU.
        Được gọi bởi timer mỗi 10ms (100Hz).
        
        Quy trình:
        ----------
        1. Kiểm tra timeout (0.5s không có dữ liệu)
        2. Đọc từ bộ đệm Serial
        3. Tìm header 0x55
        4. Phân tích gói tin (11 bytes)
        5. Cập nhật bộ đệm (acc, gyro, angle)
        6. Nếu đủ 3 loại → xuất bản
        
        Cấu trúc gói tin (11 bytes):
        ----------------------------
        Byte 0:    0x55 (Header - cố định)
        Byte 1:    Loại (0x51/0x52/0x53)
        Byte 2-3:  Dữ liệu_X (int16, little-endian)
        Byte 4-5:  Dữ liệu_Y (int16, little-endian)
        Byte 6-7:  Dữ liệu_Z (int16, little-endian)
        Byte 8-9:  Dự trữ / Nhiệt độ
        Byte 10:   Checksum (tổng byte 0-9, 8 bit thấp)
        
        Ví dụ gói tin (Gia tốc):
        ------------------------
        55 51 00 00 00 00 00 04 00 00 5A
        ↑  ↑  ↑--DL--↑ ↑--DL--↑ ↑--DL--↑ ↑Chk
        Hdr Loại  X       Y       Z
        
        Giải mã: ax=0, ay=0, az=1g (0x0400 = 1024 → 1024/32768*16 ≈ 0.5g)
        """
        
        # =====================================================================
        # ✅ KIỂM TRA TIMEOUT
        # =====================================================================
        # Nếu quá 0.5s không nhận gói tin nào → reset cờ
        # Tránh node bị "đứng" chờ mãi 1 loại dữ liệu bị thiếu
        # 
        # Ví dụ: Nhận được acc + gyro nhưng thiếu angle
        # → Không bao giờ xuất bản vì đợi angle mãi
        # → Sau 0.5s reset để bắt đầu lại chu kỳ mới
        # =====================================================================
        current_time = time.time()
        time_since_last_packet = current_time - self.last_packet_time
        
        if time_since_last_packet > 0.5:
            # Timeout! IMU ngưng gửi dữ liệu hoặc thiếu gói tin
            if self.has_acc or self.has_gyro or self.has_angle:
                # Có dữ liệu nhưng không đủ → cảnh báo
                self.get_logger().warn(
                    f'⚠️ IMU timeout ({time_since_last_packet:.1f}s) - đang reset cờ',
                    throttle_duration_sec=5.0  # Log mỗi 5s một lần
                )
                
                # Reset cờ để bắt đầu lại
                self.has_acc = False
                self.has_gyro = False
                self.has_angle = False
        
        # =====================================================================
        # ĐỌC DỮ LIỆU SERIAL
        # =====================================================================
        try:
            # Kiểm tra số bytes có sẵn trong bộ đệm
            # in_waiting: Số bytes chưa đọc trong bộ đệm nhận
            bytes_available = self.ser.in_waiting
            
            if bytes_available == 0:
                # Không có dữ liệu mới, return
                return
            
            # Đọc và xử lý tất cả gói tin có sẵn trong bộ đệm
            # Vòng lặp cho đến khi bộ đệm rỗng hoặc không đủ 11 bytes
            while self.ser.in_waiting >= 11:
                
                # ============================================================
                # BƯỚC 1: TÌM HEADER 0x55
                # ============================================================
                # Đọc 1 byte, kiểm tra xem có phải header không
                header = self.ser.read(1)
                
                if len(header) == 0:
                    # Timeout (không có dữ liệu trong 10ms)
                    break
                
                if header[0] != 0x55:
                    # Không phải header, bỏ qua và tiếp tục
                    # (Có thể là byte giữa gói tin bị lỗi)
                    continue
                
                # ============================================================
                # BƯỚC 2: ĐỌC 10 BYTES CÒN LẠI
                # ============================================================
                # Gói tin = 11 bytes tổng: 1 (header) + 10 (dữ liệu + checksum)
                data = self.ser.read(10)
                
                if len(data) != 10:
                    # Không đủ dữ liệu (timeout hoặc bộ đệm rỗng)
                    # Gói tin không hoàn chỉnh, bỏ qua
                    self.get_logger().debug('Gói tin không đầy đủ (timeout)')
                    break
                
                # ============================================================
                # BƯỚC 3: XÁC MINH CHECKSUM (Tùy chọn - có thể bỏ comment)
                # ============================================================
                # Checksum = tổng(byte 0-9) & 0xFF (8 bits thấp)
                # calculated_checksum = (header[0] + sum(data[:9])) & 0xFF
                # received_checksum = data[9]
                # 
                # if calculated_checksum != received_checksum:
                #     self.error_count += 1
                #     self.get_logger().debug(
                #         f'Lỗi checksum: tính={calculated_checksum:02X}, '
                #         f'nhận={received_checksum:02X}'
                #     )
                #     continue
                
                # ============================================================
                # BƯỚC 4: PHÂN TÍCH GÓI TIN THEO LOẠI
                # ============================================================
                data_type = data[0]  # Byte 1: Mã loại
                self.packet_count += 1
                
                # ════════════════════════════════════════════════════════════
                # LOẠI 0x51: DỮ LIỆU GIA TỐC
                # ════════════════════════════════════════════════════════════
                if data_type == 0x51:
                    """
                    Định dạng gói tin gia tốc:
                    Byte 1: 0x51 (Loại)
                    Byte 2-3: ax (int16, little-endian)
                    Byte 4-5: ay (int16, little-endian)  
                    Byte 6-7: az (int16, little-endian)
                    
                    Chuyển đổi:
                    Giá trị thô: -32768 đến +32767 (int16)
                    Phạm vi: ±16g
                    Công thức: gia tốc (m/s²) = (thô / 32768) × 16 × 9.8
                    
                    Ví dụ:
                    Thô = 2048 → (2048/32768) × 16 × 9.8 = 9.8 m/s² (1g)
                    """
                    # Phân tích 3 giá trị int16 (6 bytes tổng)
                    ax_raw = self.parse_int16(data[1:3])  # Byte 2-3
                    ay_raw = self.parse_int16(data[3:5])  # Byte 4-5
                    az_raw = self.parse_int16(data[5:7])  # Byte 6-7
                    
                    # Chuyển đổi sang m/s²
                    # Hệ số tỷ lệ: 32768 = max int16
                    # Phạm vi: ±16g
                    # Trọng lực: 9.8 m/s²
                    self.acc[0] = (ax_raw / 32768.0) * 16.0 * 9.8
                    self.acc[1] = (ay_raw / 32768.0) * 16.0 * 9.8
                    self.acc[2] = (az_raw / 32768.0) * 16.0 * 9.8
                    
                    # Đánh dấu đã nhận
                    self.has_acc = True
                    self.last_packet_time = current_time
                    
                    self.get_logger().debug(
                        f'GIA TỐC: [{self.acc[0]:.2f}, {self.acc[1]:.2f}, {self.acc[2]:.2f}] m/s²',
                        throttle_duration_sec=1.0
                    )
                
                # ════════════════════════════════════════════════════════════
                # LOẠI 0x52: DỮ LIỆU CON QUAY HỒI CHUYỂN
                # ════════════════════════════════════════════════════════════
                elif data_type == 0x52:
                    """
                    Định dạng gói tin con quay:
                    Byte 1: 0x52 (Loại)
                    Byte 2-3: wx (int16, little-endian)
                    Byte 4-5: wy (int16, little-endian)
                    Byte 6-7: wz (int16, little-endian)
                    
                    Chuyển đổi:
                    Giá trị thô: -32768 đến +32767 (int16)
                    Phạm vi: ±2000°/s
                    Công thức: vận tốc góc (rad/s) = (thô / 32768) × 2000 × (π/180)
                    
                    Ví dụ:
                    Thô = 3276 → (3276/32768) × 2000 × (π/180) = 3.49 rad/s (≈200°/s)
                    """
                    # Phân tích 3 giá trị int16
                    wx_raw = self.parse_int16(data[1:3])
                    wy_raw = self.parse_int16(data[3:5])
                    wz_raw = self.parse_int16(data[5:7])
                    
                    # Chuyển đổi sang rad/s
                    # Tỷ lệ: 32768 = max int16
                    # Phạm vi: ±2000°/s
                    # Chuyển độ sang radian: × π/180
                    DEG_TO_RAD = math.pi / 180.0
                    self.gyro[0] = (wx_raw / 32768.0) * 2000.0 * DEG_TO_RAD
                    self.gyro[1] = (wy_raw / 32768.0) * 2000.0 * DEG_TO_RAD
                    self.gyro[2] = (wz_raw / 32768.0) * 2000.0 * DEG_TO_RAD
                    
                    self.has_gyro = True
                    self.last_packet_time = current_time
                    
                    self.get_logger().debug(
                        f'CON QUAY: [{self.gyro[0]:.2f}, {self.gyro[1]:.2f}, {self.gyro[2]:.2f}] rad/s',
                        throttle_duration_sec=1.0
                    )
                
                # ════════════════════════════════════════════════════════════
                # LOẠI 0x53: DỮ LIỆU GÓC (EULER)
                # ════════════════════════════════════════════════════════════
                elif data_type == 0x53:
                    """
                    Định dạng gói tin góc:
                    Byte 1: 0x53 (Loại)
                    Byte 2-3: Roll (int16, little-endian)
                    Byte 4-5: Pitch (int16, little-endian)
                    Byte 6-7: Yaw (int16, little-endian)
                    
                    Chuyển đổi:
                    Giá trị thô: -32768 đến +32767 (int16)
                    Phạm vi: ±180° cho roll/pitch, 0-360° cho yaw
                    Công thức: góc (rad) = (thô / 32768) × π
                    
                    Góc Euler:
                    - Roll: Xoay quanh trục X (nghiêng trái/phải)
                    - Pitch: Xoay quanh trục Y (nghiêng trước/sau)
                    - Yaw: Xoay quanh trục Z (quay theo la bàn)
                    
                    Ví dụ:
                    Thô = 16384 → (16384/32768) × π = π/2 = 90° = 1.57 rad
                    """
                    # Phân tích 3 giá trị int16
                    roll_raw = self.parse_int16(data[1:3])
                    pitch_raw = self.parse_int16(data[3:5])
                    yaw_raw = self.parse_int16(data[5:7])
                    
                    # Chuyển đổi sang radian
                    # Tỷ lệ: 32768 = max int16
                    # Phạm vi: ±π cho roll/pitch, 0-2π cho yaw
                    self.angle[0] = (roll_raw / 32768.0) * math.pi
                    self.angle[1] = (pitch_raw / 32768.0) * math.pi
                    self.angle[2] = (yaw_raw / 32768.0) * math.pi
                    
                    self.has_angle = True
                    self.last_packet_time = current_time
                    
                    self.get_logger().debug(
                        f'GÓC: [{self.angle[0]:.2f}, {self.angle[1]:.2f}, {self.angle[2]:.2f}] rad',
                        throttle_duration_sec=1.0
                    )
                
                # ============================================================
                # KIỂM TRA VÀ XUẤT BẢN KHI ĐỦ DỮ LIỆU
                # ============================================================
                # Khi có đủ cả 3 loại dữ liệu → xuất bản message IMU
                if self.has_acc and self.has_gyro and self.has_angle:
                    self.publish_imu()
                    
        except serial.SerialException as e:
            self.get_logger().error(
                f'Lỗi kết nối Serial: {e}',
                throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().error(
                f'Lỗi khi đọc IMU: {e}',
                throttle_duration_sec=5.0
            )
    
    def publish_imu(self):
        """
        Xuất bản message IMU lên topic /imu/data
        
        Message format:
        ---------------
        - Header: timestamp + frame_id
        - Orientation: quaternion (chuyển từ Euler)
        - Angular velocity: rad/s
        - Linear acceleration: m/s²
        - Covariances: ma trận hiệp phương sai (ước lượng)
        """
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Gia tốc tuyến tính (m/s²)
        msg.linear_acceleration.x = self.acc[0]
        msg.linear_acceleration.y = self.acc[1]
        msg.linear_acceleration.z = self.acc[2]
        
        # Vận tốc góc (rad/s)
        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]
        
        # Hướng (quaternion chuyển từ góc Euler)
        qx, qy, qz, qw = self.euler_to_quaternion(
            self.angle[0], self.angle[1], self.angle[2]
        )
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        
        # Ma trận hiệp phương sai (ước lượng - giá trị điển hình)
        # Giá trị 0.01 = độ lệch chuẩn 0.1 đơn vị
        # Giá trị 0.0 cho các phần tử ngoài đường chéo (giả định không tương quan)
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,     # Dòng 1: σ²_ax, cov(ax,ay), cov(ax,az)
            0.0, 0.01, 0.0,     # Dòng 2: cov(ay,ax), σ²_ay, cov(ay,az)
            0.0, 0.0, 0.01      # Dòng 3: cov(az,ax), cov(az,ay), σ²_az
        ]
        
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Xuất bản message
        self.imu_pub.publish(msg)
        
        # Cập nhật thống kê
        self.publish_count += 1
        if self.publish_count % 100 == 0:
            self.get_logger().info(
                f'📊 Thống kê: Đã xuất bản {self.publish_count} message IMU '
                f'(Gói tin: {self.packet_count}, Lỗi: {self.error_count})'
            )
        
        # Reset cờ để chờ chu kỳ tiếp theo
        self.has_acc = False
        self.has_gyro = False
        self.has_angle = False
    
    def parse_int16(self, data):
        """
        Phân tích 2 bytes thành int16
        
        Args:
            data (bytes): 2 bytes dữ liệu (little-endian)
        
        Returns:
            int: Giá trị int16 (-32768 đến 32767)
        
        Format: Little-endian (byte thấp trước)
        Ví dụ: [0x00, 0x04] = 0x0400 = 1024
        """
        return struct.unpack('<h', data)[0]
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Chuyển đổi góc Euler thành Quaternion
        
        Args:
            roll (float): Góc roll (rad) - Xoay quanh trục X
            pitch (float): Góc pitch (rad) - Xoay quanh trục Y
            yaw (float): Góc yaw (rad) - Xoay quanh trục Z
        
        Returns:
            tuple: (qx, qy, qz, qw) - Quaternion
        
        Quaternion:
        -----------
        Biểu diễn hướng trong không gian 3D không bị gimbal lock
        q = w + xi + yj + zk
        Với |q| = 1 (normalized)
        
        Công thức chuyển đổi:
        --------------------
        Sử dụng công thức Euler ZYX (Yaw-Pitch-Roll)
        """
        # Tính nửa góc (half angles)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        # Tính các thành phần quaternion
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def __del__(self):
        """Destructor - Đóng kết nối Serial khi node bị hủy"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('🔌 Đã đóng kết nối Serial với IMU')


def main(args=None):
    """
    Hàm main - Điểm khởi động chương trình
    
    Quy trình:
    ----------
    1. Khởi tạo ROS2
    2. Tạo node HWT901BDriver
    3. Spin (chạy vòng lặp ROS2)
    4. Xử lý shutdown khi kết thúc
    """
    rclpy.init(args=args)
    
    try:
        # Tạo node driver
        node = HWT901BDriver()
        
        # Chạy node (vòng lặp xử lý callbacks + timer)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Người dùng nhấn Ctrl+C
        print("\n⚠️ Đã nhận Ctrl+C, đang tắt...")
        
    except Exception as e:
        # Lỗi không mong đợi
        print(f"❌ Lỗi nghiêm trọng: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup: Hủy node và tắt ROS2
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Đã tắt driver IMU")


if __name__ == '__main__':
    main()


# ==============================================================================
# HƯỚNG DẪN SỬ DỤNG VÀ KHẮC PHỤC SỰ CỐ
# ==============================================================================
"""
═══════════════════════════════════════════════════════════════════════════════
CÁCH CHẠY NODE
═══════════════════════════════════════════════════════════════════════════════

1. CHẠY RIÊNG (Standalone):
   
   ros2 run mecanum_robot_bringup hwt901b_driver.py \\
       --ros-args \\
       -p port:=/dev/ttyUSB2 \\
       -p baudrate:=9600 \\
       -p frame_id:=imu_link

2. CHẠY TRONG LAUNCH FILE:
   
   ros2 launch mecanum_robot_bringup robot_bringup.launch.py

3. KIỂM TRA HOẠT ĐỘNG:
   
   # Xem topics
   ros2 topic list | grep imu
   
   # Kiểm tra tần số
   ros2 topic hz /imu/data
   
   # Xem dữ liệu
   ros2 topic echo /imu/data
   
   # Visualize trong RViz
   rviz2
   # Add → Imu → Topic: /imu/data

═══════════════════════════════════════════════════════════════════════════════
KHẮC PHỤC SỰ CỐ
═══════════════════════════════════════════════════════════════════════════════

❌ LỖI 1: "Không thể mở cổng serial /dev/ttyUSB2"
   Nguyên nhân:
   - IMU chưa kết nối
   - Cổng USB sai
   - Không có quyền truy cập
   
   Giải pháp:
   ls -l /dev/ttyUSB*                    # Kiểm tra cổng có tồn tại
   sudo chmod 666 /dev/ttyUSB2           # Cấp quyền tạm thời
   sudo usermod -aG dialout $USER        # Cấp quyền vĩnh viễn (cần logout)

❌ LỖI 2: "IMU timeout - đang reset cờ"
   Nguyên nhân:
   - Baudrate không khớp (9600 vs 115200)
   - Kết nối Serial không ổn định
   - IMU bị lỗi phần cứng
   
   Giải pháp:
   # Kiểm tra baudrate IMU (mặc định 9600)
   # Nếu đã config lên 115200 → sửa launch file
   
   # Test kết nối:
   sudo apt install minicom
   sudo minicom -D /dev/ttyUSB2 -b 9600
   # Phải thấy dữ liệu binary hiện ra

❌ LỖI 3: "Không xuất bản message IMU"
   Nguyên nhân:
   - IMU chỉ gửi 1-2 loại dữ liệu, thiếu loại thứ 3
   - Baudrate sai
   - Output rate IMU quá thấp
   
   Giải pháp:
   # Kiểm tra có nhận gói tin không:
   ros2 topic echo /rosout | grep "GIA TỐC\|CON QUAY\|GÓC"
   
   # Nếu chỉ thấy 2/3 loại → IMU cần cấu hình lại
   # Dùng phần mềm WitMotion để bật đủ 3 loại output

❌ LỖI 4: "TF không tìm thấy transform từ base_link đến imu_link"
   Nguyên nhân:
   - URDF chưa load
   - robot_state_publisher chưa chạy
   - Frame name không khớp
   
   Giải pháp:
   ros2 run tf2_ros tf2_echo base_link imu_link
   # Phải thấy transform, nếu không:
   ros2 launch mecanum_robot_description display.launch.py

❌ LỖI 5: "Dữ liệu IMU bị nhiễu/nhảy giá trị"
   Nguyên nhân:
   - Nhiễu điện từ (motor, ESC)
   - Nguồn điện không ổn định
   - IMU cần hiệu chuẩn
   
   Giải pháp:
   # 1. Đặt IMU xa motor/ESC
   # 2. Dùng nguồn ổn áp 5V riêng cho IMU
   # 3. Hiệu chuẩn IMU bằng phần mềm WitMotion
   # 4. Thêm capacitor 100µF gần chân nguồn IMU

═══════════════════════════════════════════════════════════════════════════════
KIỂM TRA VÀ XÁC NHẬN
═══════════════════════════════════════════════════════════════════════════════

✅ CHECKLIST:

□ 1. Kết nối phần cứng:
     - IMU có nguồn 5V
     - TX IMU → RX USB-Serial
     - RX IMU → TX USB-Serial
     - GND chung

□ 2. Kiểm tra cổng Serial:
     ls -l /dev/ttyUSB*
     # Phải thấy ttyUSB0, ttyUSB1, hoặc ttyUSB2

□ 3. Kiểm tra baudrate:
     # Mặc định: 9600
     # Nếu đã config: 115200

□ 4. Chạy node:
     ros2 run mecanum_robot_bringup hwt901b_driver.py

□ 5. Kiểm tra topic:
     ros2 topic hz /imu/data
     # Phải thấy ~10Hz

□ 6. Kiểm tra dữ liệu:
     ros2 topic echo /imu/data --once
     # Phải thấy orientation, angular_velocity, linear_acceleration

□ 7. Kiểm tra trong RViz:
     # Add Imu display
     # Robot phải xoay theo IMU khi nghiêng

═══════════════════════════════════════════════════════════════════════════════
THÔNG TIN THÊM
═══════════════════════════════════════════════════════════════════════════════

📖 Tài liệu HWT901B:
    https://github.com/WITMOTION/HWT901B

🔧 Phần mềm cấu hình:
    WitMotion (Windows only)
    Download từ: https://www.wit-motion.com/

📊 Thông số điển hình:
    - Output rate: 10Hz (có thể lên 200Hz)
    - Noise: <0.01 m/s² (acc), <0.05°/s (gyro)
    - Drift: <1°/h (gyro)
    - Update rate: 100ms (10Hz)

═══════════════════════════════════════════════════════════════════════════════
"""