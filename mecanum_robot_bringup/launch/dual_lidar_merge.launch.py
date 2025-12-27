#!/usr/bin/env python3
"""
dual_lidar_merge.launch.py
==========================
Launch file khởi động 2 RPLidar A1M8 và merge dữ liệu thành 1 scan duy nhất.

"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Tạo launch description chứa 3 nodes:
    1. lidar1_node: RPLidar thứ nhất (góc trước-phải)
    2. lidar2_node: RPLidar thứ hai (góc sau-trái)
    3. laser_merger: Node merge 2 scan thành 1
    """
    
    # ==========================================================================
    # LIDAR 1 NODE - RPLidar A1M8 góc trước-phải (xoay 180° trong URDF)
    # ==========================================================================
    lidar1_node = Node(
        package='rplidar_ros',           # Package ROS2 driver cho RPLidar
        executable='rplidar_node',       # Node executable
        name='lidar1',                   # Tên node (hiển thị trong ros2 node list)
        
        # Parameters - Cấu hình cho LiDAR
        parameters=[{
            # Serial port configuration
            'serial_port': '/dev/ttyUSB0',      # Port USB của LiDAR 1
                                                 # ⚠️ Kiểm tra: ls /dev/ttyUSB*
                                                 # ⚠️ Có thể thay đổi khi cắm lại USB!
                                                 
            'serial_baudrate': 115200,           # Baudrate cho A1M8: 115200 (chuẩn)
                                                 # A2/A3: 256000
            
            # Frame configuration
            'frame_id': 'lidar1_link',           # TF frame của LiDAR này
                                                 # Phải khớp với URDF!
            
            # Scan configuration
            'angle_compensate': True,            # Bù góc khi motor quay không đều
                                                 # True = scan chính xác hơn (khuyến nghị)
                                                 
            'scan_mode': 'Standard',             # Chế độ quét:
                                                 # 'Standard': 5.5Hz, ~8000 samples/s
                                                 # 'Express': 10Hz (nếu support)
                                                 # 'Boost': Cao hơn (A2/A3)
            
            # Giới hạn góc quét (90-360°)
             'angle_min': 1.5708,                # 90° (rad) 
             'angle_max': -3.14159,              # 360° (rad) 
            
            # Range limits
             'range_min': 0.15,                 # Khoảng cách tối thiểu (m)
             'range_max': 12.0,                 # Khoảng cách tối đa (m)
        }],
        
        output='screen',                         # In log ra terminal (để debug)
        
        # Remapping - Đổi tên topic
        remappings=[
            ('scan', 'scan1')                    # /scan (mặc định) → /scan1
        ]
    )
    
    # ==========================================================================
    # LIDAR 2 NODE - RPLidar A1M8 góc sau-trái (Không xoay URDF)
    # ==========================================================================
    lidar2_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='lidar2',
        
        parameters=[{
            'serial_port': '/dev/ttyUSB1',       # Port USB của LiDAR 2
                                                 # ⚠️ Phải khác với LiDAR 1!
                                                 # ⚠️ Có thể đổi chỗ khi cắm lại!
                                                 
            'serial_baudrate': 115200,           # Baudrate giống LiDAR 1
            'frame_id': 'lidar2_link',           # TF frame khác với LiDAR 1
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            
            # Giới hạn góc quét (90-360°)
             'angle_min': 1.5708,
             'angle_max': -3.14159,
             
            # Range limits
             'range_min': 0.15,                 # Khoảng cách tối thiểu (m)
             'range_max': 12.0,                 # Khoảng cách tối đa (m)
        }],
        
        output='screen',
        
        remappings=[
            ('scan', 'scan2')                    # /scan (mặc định) → /scan2
        ]
    )
    
    # ==========================================================================
    # LASER SCAN MERGER NODE - Merge 2 scan thành 1 scan 360°
    # ==========================================================================
    # ⚠️ Node này là CUSTOM Python script trong package của bạn!
    # ⚠️ Phải đảm bảo file laser_scan_merger.py tồn tại và executable
    
    laser_merger = Node(
        package='mecanum_robot_bringup',         # Package chứa script merger
                                                 # ⚠️ Kiểm tra package name đúng!
                                                 
        executable='laser_scan_merger.py',       # Script Python merge scan
                                                 # Phải có shebang: #!/usr/bin/env python3
                                                 # Phải chmod +x laser_scan_merger.py
                                                 
        name='laser_scan_merger',                # Tên node
        
        # Parameters cho merger
        parameters=[{
            'scan1_topic': 'scan1',              # Topic input từ LiDAR 1
            'scan2_topic': 'scan2',              # Topic input từ LiDAR 2
            'output_topic': 'scan_merged',       # Topic output (scan đã merge)
            'output_frame': 'base_link',         # Frame của output scan
                                                 # ⚠️ Phải có TF: base_link → lidar1/2_link
        }],
        
        output='screen'                          # In log để debug
    )
    
    # ==========================================================================
    # RETURN LAUNCH DESCRIPTION
    # ==========================================================================
    # Thứ tự launch: lidar1 → lidar2 → merger
    # Merger sẽ đợi cả 2 scan1 và scan2 có dữ liệu mới publish
    
    return LaunchDescription([
        lidar1_node,     # Khởi động LiDAR 1
        lidar2_node,     # Khởi động LiDAR 2
        laser_merger     # Khởi động merger (sẽ subscribe scan1 + scan2)
    ])


# ==============================================================================
# CÁCH SỬ DỤNG
# ==============================================================================
"""
1. Build workspace:
   cd ~/mecanum_robot_ws
   colcon build --packages-select mecanum_robot_bringup
   source install/setup.bash

2. Kiểm tra USB ports:
   ls -l /dev/ttyUSB*
   # Kết quả mong đợi:
   # /dev/ttyUSB0
   # /dev/ttyUSB1

3. Cấp quyền cho USB (nếu cần):
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB1
   # Hoặc thêm user vào group dialout:
   sudo usermod -aG dialout $USER
   # (cần logout/login lại)

4. Launch:
   ros2 launch mecanum_robot_bringup dual_lidar_merge.launch.py

5. Kiểm tra topics:
   ros2 topic list
   # Phải thấy:
   # /scan1
   # /scan2
   # /scan_merged

6. Kiểm tra dữ liệu:
   ros2 topic echo /scan_merged
   ros2 topic hz /scan_merged

7. Visualize trong RViz:
   rviz2
   # Add → LaserScan
   # Topic: /scan_merged
   # Fixed Frame: base_link
"""

# ==============================================================================
# TROUBLESHOOTING
# ==============================================================================
"""
LỖI 1: "Could not open serial port /dev/ttyUSB0"
   → Kiểm tra: ls /dev/ttyUSB*
   → Cắm lại USB
   → Cấp quyền: sudo chmod 666 /dev/ttyUSB*

LỖI 2: "Failed to get device info"
   → Baudrate sai (phải là 115200 cho A1M8)
   → LiDAR bị lỗi phần cứng
   → Thử reset: ngắt nguồn 10s

LỖI 3: "No transform from base_link to lidar1_link"
   → URDF chưa load
   → Chạy: ros2 run robot_state_publisher robot_state_publisher

LỖI 4: "laser_scan_merger.py not found"
   → Script chưa tồn tại
   → Chưa chmod +x
   → Package name sai

LỖI 5: USB đổi chỗ (ttyUSB0 ↔ ttyUSB1)
   → Dùng udev rules để fix port
   → Hoặc dùng serial number để identify
"""