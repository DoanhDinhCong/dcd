#!/usr/bin/env python3
"""
slam_mapping.launch.py - Launch file cho SLAM (Simultaneous Localization And Mapping)
=====================================================================================
File launch khởi động SLAM Toolbox để tạo bản đồ cho robot mecanum.

CHỨC NĂNG:
----------
1. Khởi động SLAM Toolbox node (async mode)
2. Tạo bản đồ real-time khi robot di chuyển
3. Mở RViz2 để visualize quá trình mapping
4. Định vị robot đồng thời khi tạo bản đồ

SLAM TOOLBOX LÀ GÌ?
-------------------
SLAM = Simultaneous Localization And Mapping
- Localization: Xác định vị trí robot trên bản đồ
- Mapping: Tạo bản đồ môi trường xung quanh
- Simultaneous: Làm cả 2 việc cùng lúc!

KIẾN TRÚC:
----------
┌──────────────┐
│ LaserScan    │──┐
│ (/scan)      │  │
└──────────────┘  │    ┌─────────────────┐      ┌──────────┐
                  ├───→│  SLAM Toolbox   │─────→│   Map    │
┌──────────────┐  │    │  (Graph-based)  │      │ (/map)   │
│ Odometry     │──┤    └─────────────────┘      └──────────┘
│ (/odom)      │  │            ↓
└──────────────┘  │    ┌─────────────────┐
                  │    │   TF: map→odom  │
┌──────────────┐  │    └─────────────────┘
│ TF tree      │──┘
└──────────────┘

INPUT:
- /scan hoặc /scan_merged: LaserScan data
- /odom: Odometry data
- TF: base_link, laser_frame, odom

OUTPUT:
- /map: Bản đồ occupancy grid (2D)
- TF: map → odom transform
- /slam_toolbox/graph: Pose graph

CÁCH SỬ DỤNG:
-------------
# Bước 1: Chạy robot
ros2 launch mecanum_robot_bringup robot_bringup.launch.py

# Bước 2: Chạy SLAM
ros2 launch mecanum_robot_bringup slam_mapping.launch.py

# Bước 3: Điều khiển robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Bước 4: Lưu bản đồ
ros2 run nav2_map_server map_saver_cli -f my_map


"""

from launch import LaunchDescription                                    # Chứa tất cả nodes
from launch_ros.actions import Node                                     # Định nghĩa 1 node
from launch.actions import DeclareLaunchArgument                        # Tham số đầu vào
from launch.substitutions import LaunchConfiguration                    # Đọc tham số
import os                                                               # Xử lý đường dẫn
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Tạo launch description cho SLAM Mapping
    
    Returns:
        LaunchDescription: Mô tả các nodes và actions cần launch
    
    Quy trình:
    ---------
    1. Tìm đường dẫn package
    2. Load config files
    3. Declare launch arguments
    4. Launch SLAM Toolbox node
    5. Launch RViz2 với config
    """
    
    pkg_share = get_package_share_directory('mecanum_robot_bringup')
    
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    # LaunchConfiguration: Đọc giá trị từ command line
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Sử dụng thời gian simulation (True) hoặc thực tế (False)'
    )
    
    # Declare argument: SLAM params file
    # Cho phép user dùng config file khác
    declare_slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Đường dẫn đến file config SLAM Toolbox'
    )
    
    # Declare argument: RViz config
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Đường dẫn đến file config RViz2'
    )
    
    # =========================================================================
    # SLAM TOOLBOX NODE
    # =========================================================================
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        
        name='slam_toolbox',
        parameters=[
            slam_params,  # Load từ YAML file
            {'use_sim_time': use_sim_time}
        ],
        
        output='screen',
        
    )
    
    # =========================================================================
    # RVIZ2 NODE - Visualization
    # =========================================================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        
        # ⚠️ LƯU Ý: Nếu file config không tồn tại:
        # - Tạo mới: Mở RViz → Add displays → Save Config
        # - Hoặc bỏ argument '-d' để dùng default
    )
    
    # =========================================================================
    # RETURN LAUNCH DESCRIPTION
    # =========================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_arg,
        declare_slam_params_file_arg,
        declare_rviz_config_arg,
        
        # SLAM Toolbox
        slam_toolbox,
        
        # Visualization
        rviz,
        
    ])


# ==============================================================================
# HƯỚNG DẪN SỬ DỤNG CHI TIẾT
# ==============================================================================
"""
═══════════════════════════════════════════════════════════════════════════════
QUY TRÌNH HOÀN CHỈNH TẠO BẢN ĐỒ BẰNG SLAM
═══════════════════════════════════════════════════════════════════════════════

CHUẨN BỊ TRƯỚC KHI CHẠY SLAM:
─────────────────────────────
✅ 1. Kiểm tra LaserScan hoạt động:
   ros2 topic echo /scan_merged --once
   # Phải thấy dữ liệu ranges

✅ 2. Kiểm tra Odometry:
   ros2 topic echo /odom --once
   # Phải thấy pose và twist

✅ 3. Kiểm tra TF tree:
   ros2 run tf2_tools view_frames
   # Phải có: odom → base_link → laser_frame

✅ 4. Chuẩn bị môi trường:
   - Khu vực sáng (Lidar hoạt động tốt)
   - Có nhiều đặc trưng (tường, cột, góc)
   - Không quá rộng (bắt đầu với phòng nhỏ)


BƯỚC 1: KHỞI ĐỘNG HỆ THỐNG
──────────────────────────
# Terminal 1: Chạy robot (sensors, motors, TF)
ros2 launch mecanum_robot_bringup robot_bringup.launch.py

# Terminal 2: Chạy SLAM
ros2 launch mecanum_robot_bringup slam_mapping.launch.py

# Hoặc với custom config:
ros2 launch mecanum_robot_bringup slam_mapping.launch.py \\
    slam_params_file:=/path/to/custom_params.yaml


BƯỚC 2: ĐIỀU KHIỂN ROBOT TẠO BẢN ĐỒ
───────────────────────────────────
# Terminal 3: Teleop để điều khiển robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Mẹo tạo bản đồ tốt:
# 1. Di chuyển CHẬM (0.1-0.2 m/s)
# 2. Xoay CHẬM (0.3-0.5 rad/s)
# 3. Đi theo thứ tự: chu vi phòng → bên trong
# 4. Quay lại điểm xuất phát (loop closure)
# 5. Đi qua tất cả các khu vực cần map


BƯỚC 3: THEO DÕI QUÁ TRÌNH TRONG RVIZ
─────────────────────────────────────
Trong RViz, quan sát:
✅ Map đang được tạo (màu xám = unknown, trắng = free, đen = occupied)
✅ LaserScan khớp với map
✅ Robot pose (mũi tên) đúng vị trí
✅ Trajectory (đường đi) của robot

Nếu thấy:
❌ Map bị drift (lệch dần) → Odometry không chính xác
❌ LaserScan không khớp map → TF hoặc sensor sai
❌ Pose bị nhảy → Loop closure detection


BƯỚC 4: LƯU BẢN ĐỒ
──────────────────
# Cách 1: Dùng map_saver_cli (KHUYẾN NGHỊ)
ros2 run nav2_map_server map_saver_cli -f my_map

# Cách 2: Dùng SLAM Toolbox service
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
    "{filename: '/path/to/my_map'}"

# Kết quả: Tạo 2 files
# - my_map.yaml: Metadata (resolution, origin, thresholds)
# - my_map.pgm: Ảnh bản đồ (grayscale)

# Copy vào thư mục maps:
cp my_map.* ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/


BƯỚC 5: XÁC MINH BẢN ĐỒ
───────────────────────
# Xem ảnh map:
eog my_map.pgm
# Hoặc:
display my_map.pgm

# Kiểm tra YAML:
cat my_map.yaml

# Nội dung nên có:
# image: my_map.pgm
# resolution: 0.05
# origin: [-10.0, -10.0, 0.0]
# occupied_thresh: 0.65
# free_thresh: 0.196
# negate: 0


BƯỚC 6: SỬ DỤNG BẢN ĐỒ CHO NAVIGATION
────────────────────────────────────
# Chạy navigation với bản đồ vừa tạo:
ros2 launch mecanum_robot_bringup navigation.launch.py \\
    map:=/path/to/my_map.yaml

═══════════════════════════════════════════════════════════════════════════════
MẸO TẠO BẢN ĐỒ TỐT
═══════════════════════════════════════════════════════════════════════════════

✅ MẸO 1: Di chuyển chậm và ổn định
   - Vận tốc: 0.1-0.2 m/s (tuyến tính)
   - Vận tốc góc: 0.3-0.5 rad/s (xoay)
   - Tránh tăng tốc đột ngột

✅ MẸO 2: Đi theo pattern hệ thống
   1. Đi dọc tường phòng (chu vi)
   2. Đi zigzag bên trong
   3. Đi qua tất cả doorways
   4. Quay lại điểm bắt đầu

✅ MẸO 3: Loop closure
   - Quay lại điểm xuất phát
   - Đi qua cùng vị trí nhiều lần
   - Giúp SLAM optimize và giảm drift

✅ MẸO 4: Môi trường tốt cho SLAM
   ✅ Có nhiều đặc trưng: tường, cột, góc, đồ vật
   ✅ Ánh sáng tốt (nếu dùng camera SLAM)
   ✅ Bề mặt không phản chiếu (tránh gương, kính)
   ❌ Tránh: Phòng trống, hành lang dài thẳng, bề mặt trong suốt

✅ MẸO 5: Kiểm tra real-time
   - Luôn nhìn RViz khi mapping
   - Dừng lại nếu thấy map bị lệch
   - Reset và làm lại nếu cần

═══════════════════════════════════════════════════════════════════════════════
KHẮC PHỤC SỰ CỐ
═══════════════════════════════════════════════════════════════════════════════

❌ LỖI 1: "SLAM Toolbox not publishing map"
   Nguyên nhân:
   - LaserScan topic sai
   - TF không đầy đủ
   - Odometry không có
   
   Giải pháp:
   # Kiểm tra LaserScan:
   ros2 topic list | grep scan
   ros2 topic echo /scan_merged --once
   
   # Kiểm tra Odometry:
   ros2 topic echo /odom --once
   
   # Kiểm tra TF:
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo odom base_link


❌ LỖI 2: "Map drifting (lệch dần)"
   Nguyên nhân:
   - Odometry không chính xác
   - Encoder ticks sai
   - Wheel slip (bánh trượt)
   
   Giải pháp:
   # Kiểm tra odometry bằng cách:
   # 1. Cho robot đi thẳng 1m
   # 2. Xem /odom có báo đúng 1m không
   
   ros2 topic echo /odom
   
   # Nếu sai → cần tune:
   # - ticks_per_rev
   # - wheel_radius
   # - wheel_base


❌ LỖI 3: "Loop closure not working"
   Nguyên nhân:
   - Parameters loop_closure không phù hợp
   - Robot di chuyển quá nhanh
   - Môi trường thay đổi
   
   Giải pháp:
   # Sửa trong slam_toolbox_params.yaml:
   # minimum_travel_distance: 0.3
   # minimum_travel_heading: 0.3
   # loop_search_space_dimension: 8.0
   # loop_match_minimum_chain_size: 10


❌ LỖI 4: "LaserScan and map not aligned"
   Nguyên nhân:
   - TF giữa base_link và laser_frame sai
   - Thời gian không đồng bộ
   
   Giải pháp:
   # Kiểm tra TF:
   ros2 run tf2_ros tf2_echo base_link laser_frame
   
   # Phải thấy transform chính xác
   # Translation: [x, y, z] của laser trên robot
   # Rotation: Hướng của laser


❌ LỖI 5: "Map saving failed"
   Nguyên nhân:
   - Đường dẫn không có quyền ghi
   - Tên file sai
   - Thư mục không tồn tại
   
   Giải pháp:
   # Kiểm tra quyền:
   ls -ld ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/
   
   # Tạo thư mục nếu chưa có:
   mkdir -p ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/
   
   # Lưu với đường dẫn đầy đủ:
   ros2 run nav2_map_server map_saver_cli \\
       -f ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/my_map


❌ LỖI 6: "SLAM performance slow"
   Nguyên nhân:
   - CPU yếu
   - Resolution quá cao
   - Update rate quá nhanh
   
   Giải pháp:
   # Giảm resolution trong params:
   # resolution: 0.05 → 0.10
   
   # Giảm update rate:
   # map_update_interval: 5.0 → 10.0


❌ LỖI 7: "RViz config file not found"
   Nguyên nhân:
   - File slam.rviz chưa tạo
   - Đường dẫn sai
   
   Giải pháp:
   # Tạo config mới:
   ros2 run rviz2 rviz2
   # Add displays: Map, LaserScan, TF, Path
   # Save: File → Save Config As → slam.rviz
   
   # Hoặc bỏ argument config trong launch file

═══════════════════════════════════════════════════════════════════════════════
KIỂM TRA VÀ XÁC NHẬN
═══════════════════════════════════════════════════════════════════════════════

✅ CHECKLIST TRƯỚC KHI CHẠY SLAM:

□ 1. Robot sensors hoạt động:
     ros2 topic list | grep -E "scan|odom"

□ 2. LaserScan có dữ liệu:
     ros2 topic echo /scan_merged --once
     # Phải thấy ranges array

□ 3. Odometry chính xác:
     ros2 topic echo /odom
     # Cho robot đi → odom phải update

□ 4. TF tree đầy đủ:
     ros2 run tf2_tools view_frames
     # Phải có: odom → base_link → laser_frame

□ 5. Config file tồn tại:
     ls -l ~/mecanum_robot_ws/.../config/slam_toolbox_params.yaml

□ 6. Môi trường phù hợp:
     - Có tường, góc, đặc trưng
     - Không có gương, kính
     - Ánh sáng đủ

□ 7. Chạy SLAM:
     ros2 launch mecanum_robot_bringup slam_mapping.launch.py

□ 8. Điều khiển robot:
     ros2 run teleop_twist_keyboard teleop_twist_keyboard

□ 9. Theo dõi RViz:
     - Map đang được tạo
     - LaserScan khớp
     - Pose ổn định

□ 10. Lưu map:
      ros2 run nav2_map_server map_saver_cli -f my_map

═══════════════════════════════════════════════════════════════════════════════
THÔNG TIN THÊM
═══════════════════════════════════════════════════════════════════════════════

📖 SLAM Toolbox Documentation:
    https://github.com/SteveMacenski/slam_toolbox

🔧 Parameters tuning guide:
    https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md

📊 Topics quan trọng:
    /map                    - Bản đồ occupancy grid
    /slam_toolbox/graph     - Pose graph visualization
    /slam_toolbox/scan_visualization - LaserScan transformed
    /pose                   - Robot pose từ SLAM

⚡ Performance tips:
    Resolution: 0.05m = chi tiết, 0.10m = nhanh
    Map size: Nhỏ = nhanh, Lớn = chi tiết
    Update interval: Cao = mượt, Thấp = nhanh

🎯 Best practices:
    ✅ Di chuyển chậm (0.1-0.2 m/s)
    ✅ Quay lại điểm bắt đầu (loop closure)
    ✅ Kiểm tra map real-time trong RViz
    ✅ Lưu map ngay khi hoàn thành
    ✅ Backup map files (yaml + pgm)

═══════════════════════════════════════════════════════════════════════════════
"""