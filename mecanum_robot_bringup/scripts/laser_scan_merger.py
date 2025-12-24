#!/usr/bin/env python3
"""
laser_scan_merger.py - Dual LiDAR Scan Merger
==============================================
Node này merge dữ liệu từ 2 RPLidar A1M8 thành 1 scan 360° duy nhất.
2 LiDAR được gắn ở 2 góc đối diện để giảm điểm mù.

Cấu hình LiDAR:
- LiDAR 1: Góc trước-trái, xoay 180° (phủ bên trái + trước)
- LiDAR 2: Góc sau-phải, xoay 0° (phủ bên phải + sau)

Chức năng merger:
- Đồng bộ thời gian giữa 2 scan
- Transform tọa độ về base_link frame
- Xử lý vùng overlap (chồng lấp)
- Bảo toàn intensity (cường độ phản xạ)
- Lọc bỏ dữ liệu không hợp lệ

Thuật toán:
1. Nhận scan1 và scan2 (format: polar - góc + khoảng cách)
2. Kiểm tra đồng bộ thời gian (< 0.1s)
3. Transform từ lidar1_link/lidar2_link → base_link
4. Chuyển từ tọa độ cực (r, θ) → Cartesian (x, y)
5. Tính lại góc trong base_link frame
6. Merge vào mảng 360° (-180° đến +180°)
7. Xử lý vùng overlap (chọn gần nhất/trung bình/mới nhất)
8. Publish scan đã merge


"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rclpy.time import Time
from rclpy.duration import Duration


class LaserScanMerger(Node):
    """
    Merge 2 laser scan thành 1 scan 360°.
    Transform các điểm về base_link frame và xử lý vùng overlap.
    
    Workflow:
    ┌─────────────┐         ┌─────────────┐
    │   LiDAR 1   │─ scan1 ─┤             │
    │ (180° URDF) │         │   MERGER    │─ scan_merged ─> /scan_merged
    └─────────────┘         │   (TF +     │   (360° trong base_link)
    ┌─────────────┐         │    Merge)   │
    │   LiDAR 2   │─ scan2 ─┤             │
    │ (0° URDF)   │         └─────────────┘
    └─────────────┘
    """
    
    def __init__(self):
        super().__init__('laser_scan_merger')
        
        # ====================================================================
        # PARAMETERS - Các tham số cấu hình
        # ====================================================================
        
        # Input scan topics - Các topic nhận dữ liệu từ 2 LiDAR
        self.declare_parameter('scan1_topic', '/scan1')
        self.declare_parameter('scan2_topic', '/scan2')
        
        # Output merged scan topic - Topic publish scan đã merge
        self.declare_parameter('merged_topic', '/scan_merged')
        
        # Output frame - Frame tọa độ của scan output (tất cả transform về đây)
        self.declare_parameter('target_frame', 'base_link')
        
        # Merged scan parameters - Thông số của scan đã merge
        self.declare_parameter('angle_min', -3.14159)  # -π rad (-180°) - Bắt đầu từ phía sau
        self.declare_parameter('angle_max', 3.14159)   # +π rad (+180°) - Kết thúc ở phía sau
        self.declare_parameter('angle_increment', 0.0087)  # ~0.5° resolution (720 điểm cho 360°)
                                                            # Công thức: 2π / số_điểm = 6.28 / 1440 ≈ 0.00436
        
        self.declare_parameter('range_min', 0.15)  # Khoảng cách tối thiểu hợp lệ (m)
        self.declare_parameter('range_max', 12.0)  # Khoảng cách tối đa hợp lệ (m) - RPLidar A1M8 max
        
        # Time synchronization - Đồng bộ thời gian
        self.declare_parameter('time_sync_threshold', 0.05)  # Chênh lệch thời gian tối đa cho phép (s)
                                                            # Nếu scan1 và scan2 cách nhau >0.1s → không merge
        
        # Overlap handling - Xử lý vùng chồng lấp (cả 2 LiDAR cùng quét)
        self.declare_parameter('overlap_method', 'closest')  # Phương pháp:
                                                             # 'closest': Chọn điểm gần nhất
                                                             # 'average': Lấy trung bình
                                                             # 'newest': Chọn scan mới nhất
        
        # ====================================================================
        # LIDAR LOCAL ANGLE FILTER (theo frame của từng LiDAR)
        # ====================================================================
        self.declare_parameter('scan1_angle_min', -math.pi)
        self.declare_parameter('scan1_angle_max',  math.pi)

        self.declare_parameter('scan2_angle_min', -math.pi)
        self.declare_parameter('scan2_angle_max',  math.pi)

        self.scan1_angle_min = self.get_parameter('scan1_angle_min').value
        self.scan1_angle_max = self.get_parameter('scan1_angle_max').value
        self.scan2_angle_min = self.get_parameter('scan2_angle_min').value
        self.scan2_angle_max = self.get_parameter('scan2_angle_max').value


        # Get parameters - Lấy giá trị tham số
        self.scan1_topic = self.get_parameter('scan1_topic').value
        self.scan2_topic = self.get_parameter('scan2_topic').value
        self.merged_topic = self.get_parameter('merged_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.time_sync_threshold = self.get_parameter('time_sync_threshold').value
        self.overlap_method = self.get_parameter('overlap_method').value
        
        # Calculate number of points in merged scan - Tính số điểm trong scan đã merge
        # Công thức: (góc_max - góc_min) / bước_nhảy + 1
        # Ví dụ: (π - (-π)) / 0.00436 + 1 ≈ 1441 điểm cho 360°
        self.num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        # Log configuration - In ra cấu hình
        self.get_logger().info('=' * 60)
        self.get_logger().info('Laser Scan Merger Configuration:')
        self.get_logger().info(f'  Input topics: {self.scan1_topic}, {self.scan2_topic}')
        self.get_logger().info(f'  Output topic: {self.merged_topic}')
        self.get_logger().info(f'  Target frame: {self.target_frame}')
        self.get_logger().info(f'  Angle range: [{self.angle_min:.2f}, {self.angle_max:.2f}] rad')
        self.get_logger().info(f'  Angle range: [{self.angle_min*180/math.pi:.1f}°, {self.angle_max*180/math.pi:.1f}°]')
        self.get_logger().info(f'  Resolution: {self.angle_increment*180/math.pi:.2f}°')
        self.get_logger().info(f'  Points: {self.num_points}')
        self.get_logger().info(f'  Overlap method: {self.overlap_method}')
        self.get_logger().info('=' * 60)
        
        # ====================================================================
        # TF2 SETUP - Thiết lập TF transform
        # ====================================================================
        # TF buffer lưu trữ cây transform (map→odom→base_link→lidar_link)
        self.tf_buffer = Buffer()
        # TF listener lắng nghe các transform được broadcast
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ====================================================================
        # STATE VARIABLES - Biến lưu trạng thái
        # ====================================================================
        # Latest scans from each LiDAR - Scan mới nhất từ mỗi LiDAR
        self.scan1 = None  # LaserScan message từ LiDAR 1
        self.scan2 = None  # LaserScan message từ LiDAR 2
        self.scan1_time = None  # Thời điểm nhận scan1
        self.scan2_time = None  # Thời điểm nhận scan2
        
        # Statistics - Thống kê
        self.merge_count = 0        # Số lần merge thành công
        self.sync_failures = 0      # Số lần fail do không đồng bộ thời gian
        
        # ====================================================================
        # PUBLISHERS AND SUBSCRIBERS
        # ====================================================================
        
        # Subscribe to both LiDAR scans - Đăng ký nhận dữ liệu từ 2 LiDAR
        self.scan1_sub = self.create_subscription(
            LaserScan,              # Message type
            self.scan1_topic,       # Topic name: /scan1
            self.scan1_callback,    # Callback function
            10                      # QoS queue size
        )
        
        self.scan2_sub = self.create_subscription(
            LaserScan,
            self.scan2_topic,       # Topic name: /scan2
            self.scan2_callback,
            10
        )
        
        # Publish merged scan - Publisher cho scan đã merge
        self.merged_pub = self.create_publisher(
            LaserScan,              # Message type
            self.merged_topic,      # Topic name: /scan_merged
            10                      # QoS queue size
        )
        
        # Timer for periodic merging - Timer gọi hàm merge định kỳ
        # 0.1s = 10Hz → Merge và publish mỗi 100ms
        self.create_timer(0.1, self.merge_scans)
        
        self.get_logger().info('✅ Laser scan merger initialized')
    
    def scan1_callback(self, msg):
        """
        Callback khi nhận scan từ LiDAR 1.
        Chỉ lưu trữ, không xử lý ngay (xử lý trong timer).
        
        Args:
            msg (LaserScan): Scan message từ /scan1
        """
        self.scan1 = msg
        self.scan1_time = self.get_clock().now()  # Lưu timestamp
    
    def scan2_callback(self, msg):
        """
        Callback khi nhận scan từ LiDAR 2.
        
        Args:
            msg (LaserScan): Scan message từ /scan2
        """
        self.scan2 = msg
        self.scan2_time = self.get_clock().now()
    
    def merge_scans(self):
        """
        Merge 2 laser scan thành 1 scan 360°.
        Được gọi bởi timer với tần số cố định (10Hz).
        
        Quy trình:
        1. Kiểm tra có đủ dữ liệu (cả 2 scan)
        2. Kiểm tra đồng bộ thời gian
        3. Lấy TF transform (lidar_link → base_link)
        4. Chuyển scan từ polar → Cartesian
        5. Transform sang base_link frame
        6. Merge vào mảng 360°
        7. Publish scan merged
        """
        # ====================================================================
        # STEP 1: CHECK DATA AVAILABILITY - Kiểm tra có dữ liệu
        # ====================================================================
        if self.scan1 is None or self.scan2 is None:
            # Chưa nhận đủ cả 2 scan, bỏ qua lần merge này
            return
        
        # ====================================================================
        # STEP 2: CHECK TIME SYNCHRONIZATION - Kiểm tra đồng bộ thời gian
        # ====================================================================
        if self.scan1_time is None or self.scan2_time is None:
            return
        
        # Tính chênh lệch thời gian giữa 2 scan
        time_diff = abs((self.scan1_time - self.scan2_time).nanoseconds / 1e9)
        
        if time_diff > self.time_sync_threshold:
            # 2 scan cách nhau quá xa về thời gian (>0.1s)
            # Có thể do 1 LiDAR bị delay hoặc mất kết nối
            self.sync_failures += 1
            if self.sync_failures % 100 == 0:
                # Cảnh báo mỗi 100 lần fail
                self.get_logger().warn(
                    f'⚠️ Time sync issues: {self.sync_failures} failures, '
                    f'latest diff: {time_diff:.3f}s (threshold: {self.time_sync_threshold}s)'
                )
            return
        
        # ====================================================================
        # STEP 3: GET TF TRANSFORMS - Lấy transform từ TF tree
        # ====================================================================
        try:
            # Lookup transform: lidar1_link → base_link
            # Cần thiết để chuyển tọa độ điểm từ frame LiDAR về frame robot
            transform1 = self.tf_buffer.lookup_transform(
                self.target_frame,              # Target: base_link
                self.scan1.header.frame_id,     # Source: lidar1_link
                self.scan1.header.stamp,                           # Thời gian: Latest (mới nhất)
                Duration(seconds=0.1)            # Timeout: 0.1s
            )
            
            # Lookup transform: lidar2_link → base_link
            transform2 = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.scan2.header.frame_id,     # Source: lidar2_link
                self.scan2.header.stamp, 
                Duration(seconds=0.1)
            )
            
        except tf2_ros.LookupException as e:
            # Transform không tồn tại trong TF tree
            # Có thể do: URDF chưa load, robot_state_publisher chưa chạy
            self.get_logger().debug(f'❌ TF lookup failed: {e}')
            return
            
        except tf2_ros.ExtrapolationException as e:
            # Transform quá cũ hoặc quá mới so với dữ liệu
            self.get_logger().debug(f'❌ TF extrapolation failed: {e}')
            return
        
        # ====================================================================
        # STEP 4: CONVERT SCANS TO CARTESIAN POINTS
        # ====================================================================
        # Chuyển từ polar (r, θ) trong lidar_frame → Cartesian (x, y) trong base_link
        # scan_to_points() trả về list các điểm: [(x, y), intensity]
        points1 = self.scan_to_points(
            self.scan1,
            transform1,
            self.scan1_angle_min,
            self.scan1_angle_max
        )

        points2 = self.scan_to_points(
            self.scan2,
            transform2,
            self.scan2_angle_min,
            self.scan2_angle_max
        )

        
        # ====================================================================
        # STEP 5: INITIALIZE MERGED ARRAYS - Khởi tạo mảng merge
        # ====================================================================
        # Mảng chứa khoảng cách cho mỗi góc trong scan 360°
        # Ban đầu = inf (vô cực) = chưa có dữ liệu
        merged_ranges = np.full(self.num_points, float('inf'))
        
        # Mảng chứa intensity (cường độ phản xạ) cho mỗi góc
        merged_intensities = np.zeros(self.num_points)
        
        # ====================================================================
        # STEP 6: MERGE POINTS FROM SCAN 1
        # ====================================================================
        # Duyệt qua tất cả các điểm từ LiDAR 1
        for point, intensity in points1:
            if point is None:
                # Điểm không hợp lệ (inf, nan, hoặc ngoài range)
                continue
            
            # Tính góc của điểm này trong base_link frame
            # atan2(y, x) trả về góc từ -π đến +π
            angle = math.atan2(point[1], point[0])
            
            # Chuyển góc thành index trong mảng merged_ranges
            # Ví dụ: angle = 0° → index ở giữa mảng
            #        angle = 90° → index ở đầu mảng (nếu angle_min = -180°)
            index = self.angle_to_index(angle)
            
            if 0 <= index < self.num_points:
                # Index hợp lệ, nằm trong mảng
                
                # Tính khoảng cách từ gốc tọa độ đến điểm
                range_val = math.sqrt(point[0]**2 + point[1]**2)
                
                # Cập nhật điểm vào mảng merged
                # update_point() xử lý trường hợp có nhiều điểm cùng 1 góc (overlap)
                if self.update_point(merged_ranges, merged_intensities, 
                                   index, range_val, intensity):
                    pass  # Điểm đã được cập nhật
        
        # ====================================================================
        # STEP 7: MERGE POINTS FROM SCAN 2
        # ====================================================================
        # Tương tự scan 1
        for point, intensity in points2:
            if point is None:
                continue
            
            angle = math.atan2(point[1], point[0])
            index = self.angle_to_index(angle)
            
            if 0 <= index < self.num_points:
                range_val = math.sqrt(point[0]**2 + point[1]**2)
                
                # Cập nhật điểm vào mảng
                # Nếu index này đã có điểm từ scan1 → xử lý overlap
                self.update_point(merged_ranges, merged_intensities, 
                                index, range_val, intensity)
        
        # ====================================================================
        # STEP 8: CREATE AND PUBLISH MERGED SCAN MESSAGE
        # ====================================================================
        merged_scan = LaserScan()
        
        # Header
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        merged_scan.header.frame_id = self.target_frame  # base_link
        
        # Scan parameters - Thông số scan
        merged_scan.angle_min = self.angle_min          # -π (-180°)
        merged_scan.angle_max = self.angle_max          # +π (+180°)
        merged_scan.angle_increment = self.angle_increment  # ~0.25°
        
        merged_scan.time_increment = 0.0    # Giả định tất cả điểm được capture đồng thời
                                            # (Thực tế LiDAR quét tuần tự, nhưng sau khi merge coi như đồng thời)
        
        merged_scan.scan_time = 0.1         # Thời gian 1 scan hoàn chỉnh (s)
                                            # A1M8: ~0.1-0.18s (5.5-10Hz)
        
        merged_scan.range_min = self.range_min  # 0.15m
        merged_scan.range_max = self.range_max  # 12.0m
        
        # Data - Dữ liệu scan
        merged_scan.ranges = merged_ranges.tolist()          # Chuyển numpy array → list
        merged_scan.intensities = merged_intensities.tolist()
        
        # Publish merged scan
        self.merged_pub.publish(merged_scan)
        
        # Update statistics - Cập nhật thống kê
        self.merge_count += 1
        if self.merge_count % 100 == 0:
            # Log mỗi 100 lần merge (mỗi 5 giây nếu 20Hz)
            self.get_logger().debug(f'✅ Merged {self.merge_count} scans')
    
    def scan_to_points(self, scan, transform, angle_min_filter, angle_max_filter):

        """
        Chuyển laser scan từ tọa độ cực (polar) sang Cartesian và transform sang target frame.
        
        Quy trình cho mỗi điểm trong scan:
        1. Lấy góc: θ = angle_min + i × angle_increment
        2. Lấy khoảng cách: r = ranges[i]
        3. Chuyển sang Cartesian trong sensor frame: (x, y) = (r×cos(θ), r×sin(θ))
        4. Transform sang base_link: (x', y') = T × (x, y)
        
        Args:
            scan (LaserScan): Input scan (từ /scan1 hoặc /scan2)
            transform (TransformStamped): Transform từ sensor frame → target frame
                                         Chứa: translation (x, y, z) + rotation (quaternion)
            
        Returns:
            list: List of tuples: [((x, y), intensity), ...]
                  x, y: Tọa độ Cartesian trong base_link
                  intensity: Cường độ phản xạ
                  None nếu điểm không hợp lệ
        
        Ví dụ:
            Input scan (polar):
                angle_min = 45° (0.785 rad)
                angle_increment = 1°
                ranges = [1.0, 1.5, 2.0, inf, 3.0, ...]
                
            Output points (Cartesian trong base_link):
                [(0.7, 0.7), (1.0, 1.0), (1.4, 1.4), None, (2.1, 2.1), ...]
                  ↑ Điểm 0    ↑ Điểm 1    ↑ Điểm 2   ↑ Invalid  ↑ Điểm 4
        """
        points = []
        
        # Duyệt qua tất cả các điểm trong scan
        for i, range_val in enumerate(scan.ranges):
            
            # ================================================================
            # STEP 1: VALIDATE RANGE - Kiểm tra khoảng cách hợp lệ
            # ================================================================
            # Bỏ qua các giá trị không hợp lệ
            if math.isinf(range_val) or math.isnan(range_val):
                # inf = không phát hiện được (quá xa hoặc không có vật)
                # nan = dữ liệu lỗi
                points.append((None, 0))
                continue
            
            # Kiểm tra trong khoảng min-max
            if range_val < scan.range_min or range_val > scan.range_max:
                points.append((None, 0))
                continue
            
            # ================================================================
            # STEP 2: CALCULATE ANGLE - Tính góc của điểm này
            # ================================================================
            # Công thức: θ_i = angle_min + i × angle_increment
            angle = scan.angle_min + i * scan.angle_increment
            
            # ================================================================
            # LIDAR ANGLE FILTER (theo frame LiDAR, KHÔNG phải base_link)
            # ================================================================
            # Normalize angle về [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Bỏ điểm nếu ngoài sector cho phép của LiDAR
            if angle < angle_min_filter or angle > angle_max_filter:
                points.append((None, 0))
                continue


            # ================================================================
            # STEP 3: POLAR TO CARTESIAN - Chuyển tọa độ cực sang Cartesian
            # ================================================================
            # Trong sensor frame (lidar_link):
            # x = r × cos(θ)  → hướng về phía trước LiDAR (0°)
            # y = r × sin(θ)  → hướng sang trái LiDAR (90°)
            x_sensor = range_val * math.cos(angle)
            y_sensor = range_val * math.sin(angle)
            
            # ================================================================
            # STEP 4: CREATE POINT MESSAGE - Tạo message PointStamped
            # ================================================================
            # PointStamped = Point + Header (frame_id + timestamp)
            point_sensor = PointStamped()
            point_sensor.header.frame_id = scan.header.frame_id  # lidar1_link hoặc lidar2_link
            point_sensor.point.x = x_sensor
            point_sensor.point.y = y_sensor
            point_sensor.point.z = 0.0  # LiDAR 2D nên z = 0
            
            # ================================================================
            # STEP 5: TRANSFORM TO TARGET FRAME - Transform sang base_link
            # ================================================================
            try:
                # Áp dụng transform: point_target = T × point_sensor
                # Transform bao gồm:
                #   - Translation: dịch chuyển (x, y, z từ URDF)
                #   - Rotation: xoay (quaternion từ URDF rpy)
                point_target = tf2_geometry_msgs.do_transform_point(
                    point_sensor,   # Point trong sensor frame
                    transform       # Transform: sensor → base_link
                )
                
                # Lấy intensity nếu có
                intensity = scan.intensities[i] if len(scan.intensities) > i else 0.0
                
                # Thêm vào list: ((x, y), intensity)
                points.append((
                    [point_target.point.x, point_target.point.y],
                    intensity
                ))
                
            except Exception as e:
                # Transform failed - có thể do TF không đủ dữ liệu
                self.get_logger().debug(f'Transform failed for point {i}: {e}')
                points.append((None, 0))
        
        return points
    
    def angle_to_index(self, angle):
        """
        Chuyển đổi góc (rad) thành index trong mảng merged scan.
        
        Công thức:
            index = (angle - angle_min) / angle_increment
        
        Ví dụ:
            angle_min = -π (-180°)
            angle_max = +π (+180°)
            angle_increment = 0.00436 (~0.25°)
            num_points = 1441
            
            angle = -π → index = 0 (phía sau robot)
            angle = 0  → index = 720 (phía trước robot)
            angle = +π → index = 1440 (phía sau robot)
        
        Args:
            angle (float): Góc trong base_link frame (rad)
                          Khoảng [-π, +π] (ngược chiều kim đồng hồ từ trục X)
            
        Returns:
            int: Index trong mảng merged_ranges (0 đến num_points-1)
        
        Lưu ý:
            - Góc được normalize về [-π, +π] trước khi tính
            - Index có thể < 0 hoặc >= num_points nếu góc ngoài range
        """
        # ====================================================================
        # NORMALIZE ANGLE - Chuẩn hóa góc về [-π, +π]
        # ====================================================================
        # Ví dụ: angle = 4π → 0, angle = -3π → π
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        
        # ====================================================================
        # CALCULATE INDEX - Tính index
        # ====================================================================
        # Công thức tuyến tính: map góc → index
        # (angle - angle_min) = offset từ góc bắt đầu
        # / angle_increment = số bước nhảy từ đầu
        index = int((angle - self.angle_min) / self.angle_increment)
        
        return index
    
    def update_point(self, ranges, intensities, index, new_range, new_intensity):
        """
        Cập nhật điểm vào mảng merged scan.
        Xử lý trường hợp overlap (nhiều điểm cùng 1 góc) dựa trên phương pháp đã config.
        
        Overlap xảy ra khi:
        - Cả 2 LiDAR cùng quét 1 vùng (vùng chồng lấp ~45°)
        - Cần quyết định: chọn điểm nào hoặc kết hợp ra sao?
        
        Các phương pháp:
        
        1. 'closest' (Gần nhất) - Mặc định, an toàn:
           ┌─────────────────────────────────────┐
           │ Chọn điểm gần hơn                   │
           │ Ưu: An toàn cho navigation          │
           │ Nhược: Có thể bỏ qua vật xa         │
           └─────────────────────────────────────┘
           
        2. 'average' (Trung bình):
           ┌─────────────────────────────────────┐
           │ Lấy trung bình 2 điểm               │
           │ Ưu: Giảm nhiễu                      │
           │ Nhược: Có thể tạo điểm ảo           │
           └─────────────────────────────────────┘
           
        3. 'newest' (Mới nhất):
           ┌─────────────────────────────────────┐
           │ Luôn dùng điểm mới nhất             │
           │ Ưu: Cập nhật real-time              │
           │ Nhược: Không xử lý nhiễu            │
           └─────────────────────────────────────┘
        
        Args:
            ranges (np.array): Mảng khoảng cách đã merge (in-place update)
            intensities (np.array): Mảng intensity đã merge (in-place update)
            index (int): Index cần cập nhật trong mảng
            new_range (float): Khoảng cách mới (m)
            new_intensity (float): Intensity mới (0-255 hoặc normalized)
            
        Returns:
            bool: True nếu điểm được cập nhật, False nếu bỏ qua
        
        Ví dụ overlap:
            LiDAR 1 quét góc 90°: range = 2.0m
            LiDAR 2 quét góc 90°: range = 1.8m
            
            Method 'closest': Chọn 1.8m (gần hơn)
            Method 'average': Chọn 1.9m (trung bình)
            Method 'newest': Chọn scan được xử lý sau
        """
        # Lấy giá trị hiện tại tại index này
        current_range = ranges[index]
        
        # ====================================================================
        # OVERLAP METHOD: CLOSEST - Chọn điểm gần nhất
        # ====================================================================
        if self.overlap_method == 'closest':
            # So sánh: điểm mới có gần hơn điểm hiện tại không?
            if new_range < current_range:
                # Điểm mới gần hơn → cập nhật
                ranges[index] = new_range
                intensities[index] = new_intensity
                return True
            # Ngược lại: giữ nguyên điểm cũ (gần hơn)
            return False
                
        # ====================================================================
        # OVERLAP METHOD: AVERAGE - Lấy trung bình
        # ====================================================================
        elif self.overlap_method == 'average':
            if not math.isinf(current_range):
                # Đã có giá trị → tính trung bình
                ranges[index] = (current_range + new_range) / 2.0
                intensities[index] = (intensities[index] + new_intensity) / 2.0
            else:
                # Chưa có giá trị (inf) → gán giá trị đầu tiên
                ranges[index] = new_range
                intensities[index] = new_intensity
            return True
            
        # ====================================================================
        # OVERLAP METHOD: NEWEST - Luôn dùng mới nhất
        # ====================================================================
        elif self.overlap_method == 'newest':
            # Không quan tâm giá trị cũ, luôn ghi đè
            ranges[index] = new_range
            intensities[index] = new_intensity
            return True
        
        # Unknown method
        return False


def main(args=None):
    """
    Main entry point - Điểm khởi động chương trình.
    
    Workflow:
    1. Khởi tạo ROS2
    2. Tạo LaserScanMerger node
    3. Spin (chạy vòng lặp ROS2)
    4. Xử lý shutdown
    """
    # Khởi tạo ROS2 context
    rclpy.init(args=args)
    
    try:
        # Tạo node merger
        node = LaserScanMerger()
        
        # Chạy node (vòng lặp xử lý callbacks + timer)
        # Sẽ chạy mãi cho đến khi Ctrl+C hoặc lỗi
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Người dùng nhấn Ctrl+C
        print("\n⚠️ Keyboard interrupt, shutting down...")
        
    except Exception as e:
        # Lỗi không mong đợi
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup: Hủy node và shutdown ROS2
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ==============================================================================
# HƯỚNG DẪN SỬ DỤNG VÀ TROUBLESHOOTING
# ==============================================================================
"""
═══════════════════════════════════════════════════════════════════════════════
CÁCH CHẠY NODE
═══════════════════════════════════════════════════════════════════════════════

1. STANDALONE MODE (Chạy riêng):
   
   ros2 run mecanum_robot_bringup laser_scan_merger.py \\
       --ros-args \\
       -p scan1_topic:=/scan1 \\
       -p scan2_topic:=/scan2 \\
       -p merged_topic:=/scan_merged \\
       -p overlap_method:=closest

2. LAUNCH FILE MODE (Trong dual_lidar_merge.launch.py):
   
   ros2 launch mecanum_robot_bringup dual_lidar_merge.launch.py

3. KIỂM TRA HOẠT ĐỘNG:
   
   # Xem topics
   ros2 topic list | grep scan
   
   # Kiểm tra frequency
   ros2 topic hz /scan_merged
   
   # Xem dữ liệu
   ros2 topic echo /scan_merged --once
   
   # Visualize trong RViz
   rviz2
   # Add → LaserScan → Topic: /scan_merged

═══════════════════════════════════════════════════════════════════════════════
TROUBLESHOOTING
═══════════════════════════════════════════════════════════════════════════════

❌ LỖI 1: "No messages received from /scan1 or /scan2"
   Nguyên nhân:
   - LiDAR chưa chạy
   - Topic name sai
   
   Giải pháp:
   ros2 topic list | grep scan      # Kiểm tra topics có tồn tại
   ros2 topic echo /scan1 --once    # Kiểm tra dữ liệu

❌ LỖI 2: "TF lookup failed: Could not find transform"
   Nguyên nhân:
   - URDF chưa load
   - robot_state_publisher chưa chạy
   - Frame name sai
   
   Giải pháp:
   ros2 run tf2_ros tf2_echo base_link lidar1_link  # Kiểm tra TF
   ros2 run tf2_tools view_frames                   # Xem TF tree
   
❌ LỖI 3: "Time sync issues: X failures"
   Nguyên nhân:
   - 1 LiDAR bị delay
   - LiDAR scan rate khác nhau
   - CPU quá tải
   
   Giải pháp:
   ros2 topic hz /scan1             # Kiểm tra frequency
   ros2 topic hz /scan2
   # Tăng time_sync_threshold lên 0.2s

❌ LỖI 4: "Merged scan has many inf values"
   Nguyên nhân:
   - Góc quét của 2 LiDAR không đủ phủ 360°
   - LiDAR bị che
   - Transform sai
   
   Giải pháp:
   # Kiểm tra góc quét trong RViz
   # Kiểm tra URDF: góc xoay LiDAR đúng không?

═══════════════════════════════════════════════════════════════════════════════
OPTIMIZATION TIPS
═══════════════════════════════════════════════════════════════════════════════

🚀 TỐI ƯU HIỆU SUẤT:

1. Giảm resolution nếu CPU yếu:
   angle_increment: 0.0087  # 0.5° thay vì 0.25°
   → Giảm từ 1440 điểm xuống 720 điểm

2. Tăng timer period nếu không cần 20Hz:
   self.create_timer(0.1, self.merge_scans)  # 10Hz thay vì 20Hz

3. Dùng 'closest' overlap method (nhanh nhất):
   overlap_method: 'closest'
   # 'average' và 'newest' tốn thêm phép tính

4. Giảm time_sync_threshold nếu LiDAR ổn định:
   time_sync_threshold: 0.05  # 50ms thay vì 100ms
   → Merge chính xác hơn về thời gian

═══════════════════════════════════════════════════════════════════════════════
TESTING & VALIDATION
═══════════════════════════════════════════════════════════════════════════════

✅ CHECKLIST KIỂM TRA:

□ 1. Topics published correctly:
     ros2 topic hz /scan_merged      # Should be ~20Hz

□ 2. Full 360° coverage:
     # Trong RViz, quay robot → scan phải phủ toàn bộ xung quanh

□ 3. No ghost points:
     # Không có điểm ảo (điểm xuất hiện không có vật)

□ 4. Overlap handled correctly:
     # Trong vùng overlap, không có điểm nhảy giá trị

□ 5. TF transforms correct:
     ros2 run tf2_tools view_frames
     # Kiểm tra: base_link → lidar1_link, lidar2_link

□ 6. Performance acceptable:
     top                            # CPU usage < 20%
     ros2 topic hz /scan_merged     # Stable ~20Hz

═══════════════════════════════════════════════════════════════════════════════
"""