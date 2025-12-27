#!/usr/bin/env python3
"""
navigation.launch.py - Launch file cho Navigation Stack (Nav2)
==============================================================
File launch khởi động hệ thống navigation cho robot mecanum.

CHỨC NĂNG:
----------
1. Khởi động Nav2 stack với cấu hình tùy chỉnh
2. Load bản đồ (map) từ file YAML
3. Mở RViz2 để visualize và điều khiển
4. Cung cấp các tham số cấu hình qua launch arguments

KIẾN TRÚC NAV2:
--------------
┌─────────────────────────────────────────────────────────────┐
│                      NAV2 STACK                             │
├─────────────────────────────────────────────────────────────┤
│ Map Server  → Load và serve bản đồ                         │
│ AMCL        → Định vị robot trên bản đồ (localization)     │
│ Planner     → Lập kế hoạch đường đi toàn cục               │
│ Controller  → Điều khiển chuyển động cục bộ                │
│ BT Navigator→ Điều phối các behavior tree                  │
│ Recoveries  → Xử lý khi robot bị kẹt                       │
└─────────────────────────────────────────────────────────────┘
         ↓ publish /cmd_vel
    [velocity_bridge] → STM32 → Motors

YÊU CẦU TRƯỚC KHI CHẠY:
----------------------
1. Đã có bản đồ (chạy SLAM trước để tạo map)
2. Đã chạy robot_bringup.launch.py (TF tree, sensors)
3. Đã cấu hình nav2_params.yaml
4. Robot đã được đặt đúng vị trí trên bản đồ

CÁCH SỬ DỤNG:
-------------
# Mặc định (dùng my_map.yaml):
ros2 launch mecanum_robot_bringup navigation.launch.py

# Chỉ định bản đồ khác:
ros2 launch mecanum_robot_bringup navigation.launch.py \\
    map:=/path/to/your_map.yaml

# Với namespace (nếu có nhiều robot):
ros2 launch mecanum_robot_bringup navigation.launch.py \\
    namespace:=robot1

TROUBLESHOOTING:
---------------
- Nếu Nav2 crash: Kiểm tra TF tree (ros2 run tf2_tools view_frames)
- Nếu robot không di chuyển: Kiểm tra /cmd_vel (ros2 topic echo /cmd_vel)
- Nếu AMCL không hoạt động: Kiểm tra /scan topic
- Nếu map không load: Kiểm tra đường dẫn file map


"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Tạo launch description cho Navigation Stack
    
    Returns:
        LaunchDescription: Mô tả các nodes và actions cần launch
    
    Quy trình:
    ---------
    1. Tìm đường dẫn package
    2. Load config files (nav2_params.yaml, map.yaml)
    3. Declare launch arguments
    4. Include nav2_bringup launch file
    5. Launch RViz2 với config
    """
    
    # =========================================================================
    # TÌM ĐƯỜNG DẪN PACKAGE
    # =========================================================================
    
    # Đường dẫn tới package mecanum_robot_bringup
    # Chứa config files và maps
    pkg_share = get_package_share_directory('mecanum_robot_bringup')
    
    # Đường dẫn tới package nav2_bringup
    # Chứa các launch files của Nav2 stack
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # =========================================================================
    # ĐƯỜNG DẪN CONFIG FILES
    # =========================================================================
    
    # File cấu hình Nav2 (chứa parameters cho tất cả Nav2 nodes)
    # Bao gồm: controller, planner, behavior tree, recovery, v.v.
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # File bản đồ mặc định
    # Format: YAML với thông tin về file .pgm và metadata
    # Ví dụ nội dung:
    #   image: my_map.pgm
    #   resolution: 0.05
    #   origin: [-10.0, -10.0, 0.0]
    #   occupied_thresh: 0.65
    #   free_thresh: 0.196
    default_map = '/home/dcd/mecanum_robot_ws/src/mecanum_robot_bringup/maps/mapt8.yaml'
    
    # ✅ CẢI TIẾN: THÊM RVIZ CONFIG
    # RViz config file để tự động hiển thị robot, map, path, v.v.
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
    # Lưu ý: Cần tạo file này nếu chưa có!
    
    # =========================================================================
    # LAUNCH ARGUMENTS - Tham số có thể thay đổi từ command line
    # =========================================================================
    
    # LaunchConfiguration: Đọc giá trị argument từ command line
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ✅ CẢI TIẾN: THÊM RVIZ CONFIG ARGUMENT
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    # Declare argument: map file path
    # Cho phép user chỉ định bản đồ khác từ command line
    # Ví dụ: map:=/path/to/office_map.yaml
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Đường dẫn đầy đủ tới file YAML của bản đồ'
    )
    
    # Declare argument: use simulation time
    # True: Dùng thời gian từ /clock (Gazebo)
    # False: Dùng thời gian thực (robot thật)
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Sử dụng thời gian simulation (True) hoặc thực tế (False)'
    )
    
    # ✅ CẢI TIẾN: DECLARE RVIZ CONFIG ARGUMENT
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Đường dẫn đến file config RViz2'
    )
    
    # =========================================================================
    # NAV2 BRINGUP - Include launch file của Nav2 stack
    # =========================================================================
    
    nav2_bringup = IncludeLaunchDescription(
        # Tìm file launch
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        
        # Truyền arguments vào launch file
        # .items() chuyển dict thành list of tuples
        launch_arguments={
            # Đường dẫn bản đồ
            'map': map_file,
            
            # File cấu hình Nav2
            # Chứa tất cả parameters cho các nodes
            'params_file': nav2_params,
            
            # Sử dụng thời gian simulation hay thực tế
            # 'False' cho robot thật, 'True' cho Gazebo
            'use_sim_time': use_sim_time,
            
            # ✅ CẢI TIẾN: CÓ THỂ THÊM CÁC ARGUMENTS KHÁC
            'autostart': 'True',  # Tự động start lifecycle nodes
            # 'use_composition': 'False',  # Dùng composition hay không
            # 'use_respawn': 'False',  # Respawn nodes nếu crash
        }.items()
    )
    
    # =========================================================================
    # RVIZ2 NODE - Visualization và điều khiển
    # =========================================================================
    
    # Launch RViz2 để:
    # - Visualize robot, bản đồ, laser scan, path
    # - Gửi goal (2D Nav Goal) để điều khiển robot
    # - Xem trạng thái navigation
    # - Debug các vấn đề (costmap, path, v.v.)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        
        # ✅ CẢI TIẾN: THÊM CONFIG FILE
        # Nếu không có config → RViz mở với màn hình trống
        # Cần add display thủ công: Map, RobotModel, LaserScan, Path, v.v.
        arguments=['-d', rviz_config_file],
        
        # Parameters
        parameters=[{
            'use_sim_time': use_sim_time
        }]
        
        # ⚠️ LƯU Ý: Nếu file config không tồn tại, RViz sẽ crash!
        # Giải pháp:
        # 1. Tạo config bằng cách:
        #    - Mở RViz, add displays (Map, RobotModel, etc.)
        #    - File → Save Config As → navigation.rviz
        #    - Copy vào ~/mecanum_robot_ws/.../config/
        # 2. Hoặc bỏ argument '-d' để dùng config mặc định
    )
    
    # =========================================================================
    # ✅ CẢI TIẾN TÙY CHỌN: STATIC TRANSFORM PUBLISHER
    # =========================================================================
    # Nếu cần publish transform tĩnh (ví dụ: map → odom ban đầu)
    # Thường không cần vì AMCL sẽ publish
    
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_odom_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )
    
    # =========================================================================
    # RETURN LAUNCH DESCRIPTION
    # =========================================================================
    
    # Thứ tự launch:
    # 1. Declare arguments (để parse từ command line)
    # 2. Nav2 bringup (khởi động Nav2 stack)
    # 3. RViz (visualization)
    return LaunchDescription([
        # Arguments
        declare_map_arg,
        declare_use_sim_time_arg,
        declare_rviz_config_arg,  # ✅ Thêm argument mới
        
        # Nav2 stack
        nav2_bringup,
        
        # Visualization
        rviz,
        
        # ✅ Có thể thêm static_tf nếu cần
        # static_tf,
    ])


# ==============================================================================
# HƯỚNG DẪN SỬ DỤNG CHI TIẾT
# ==============================================================================
"""
═══════════════════════════════════════════════════════════════════════════════
QUY TRÌNH HOÀN CHỈNH ĐỂ CHẠY NAVIGATION
═══════════════════════════════════════════════════════════════════════════════

BƯỚC 1: TẠO BẢN ĐỒ (Chỉ làm 1 lần)
───────────────────────────────────
# Terminal 1: Chạy robot
ros2 launch mecanum_robot_bringup robot_bringup.launch.py

# Terminal 2: Chạy SLAM để tạo bản đồ
ros2 launch mecanum_robot_bringup slam.launch.py

# Terminal 3: Điều khiển robot đi khắp khu vực
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Lưu bản đồ khi đã quét đủ
ros2 run nav2_map_server map_saver_cli -f ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/my_map

# Kết quả: my_map.yaml và my_map.pgm trong thư mục maps/


BƯỚC 2: CHẠY NAVIGATION (Mỗi lần sử dụng)
─────────────────────────────────────────
# Terminal 1: Chạy robot (sensors, motors, TF)
ros2 launch mecanum_robot_bringup robot_bringup.launch.py

# Terminal 2: Chạy navigation
ros2 launch mecanum_robot_bringup navigation.launch.py

# Hoặc với bản đồ khác:
ros2 launch mecanum_robot_bringup navigation.launch.py \\
    map:=/path/to/office_map.yaml


BƯỚC 3: SỬ DỤNG RVIZ ĐỂ ĐIỀU KHIỂN
──────────────────────────────────
1. Trong RViz, click "2D Pose Estimate" (toolbar)
2. Click và kéo trên map để set vị trí ban đầu của robot
   → AMCL sẽ localize robot
3. Click "2D Nav Goal" (toolbar)
4. Click và kéo trên map để set điểm đích
   → Nav2 sẽ lập kế hoạch và di chuyển robot


BƯỚC 4: TẠO RVIZ CONFIG (Chỉ làm 1 lần)
───────────────────────────────────────
# Nếu RViz mở với màn hình trống:
1. Add displays:
   - RobotModel (topic: /robot_description)
   - Map (topic: /map)
   - LaserScan (topic: /scan_merged)
   - Path (topic: /plan)
   - Path (topic: /local_plan)
   - TF (để xem TF tree)
   - Odometry (topic: /odom)
   - Pose (topic: /pose)
   
2. Set Fixed Frame: "map"

3. Save config:
   File → Save Config As → 
   ~/mecanum_robot_ws/src/mecanum_robot_bringup/config/navigation.rviz

═══════════════════════════════════════════════════════════════════════════════
KHẮC PHỤC SỰ CỐ
═══════════════════════════════════════════════════════════════════════════════

❌ LỖI 1: "Could not load map"
   Nguyên nhân:
   - File map không tồn tại
   - Đường dẫn sai
   - File .pgm không có
   
   Giải pháp:
   ls -l ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/
   # Phải thấy: my_map.yaml và my_map.pgm
   
   # Kiểm tra nội dung yaml:
   cat ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/my_map.yaml


❌ LỖI 2: "No transform from map to base_link"
   Nguyên nhân:
   - URDF chưa load
   - robot_state_publisher chưa chạy
   - Chưa set initial pose trong RViz
   
   Giải pháp:
   # Kiểm tra TF tree:
   ros2 run tf2_tools view_frames
   
   # Phải thấy: map → odom → base_link → các frame khác
   
   # Nếu thiếu → chạy robot_bringup trước:
   ros2 launch mecanum_robot_bringup robot_bringup.launch.py


❌ LỖI 3: "AMCL not localizing"
   Nguyên nhân:
   - Chưa set initial pose
   - LaserScan không khớp với map
   - Robot ở vị trí khác với map
   
   Giải pháp:
   # Kiểm tra laser scan:
   ros2 topic echo /scan_merged --once
   
   # Set initial pose chính xác trong RViz:
   # - Đặt robot ở vị trí thật
   # - Click "2D Pose Estimate"
   # - Click đúng vị trí robot trên map
   # - Kéo để set hướng


❌ LỖI 4: "Robot not moving to goal"
   Nguyên nhân:
   - /cmd_vel không được subscribe
   - velocity_bridge chưa chạy
   - Parameters nav2 sai
   
   Giải pháp:
   # Kiểm tra cmd_vel:
   ros2 topic echo /cmd_vel
   
   # Phải thấy velocity commands khi set goal
   
   # Kiểm tra velocity_bridge:
   ros2 topic list | grep cmd_vel
   ros2 node list | grep velocity_bridge


❌ LỖI 5: "RViz config file not found"
   Nguyên nhân:
   - File navigation.rviz chưa tạo
   - Đường dẫn sai
   
   Giải pháp:
   # Tạo config mới:
   ros2 run rviz2 rviz2
   # Add displays → Save config
   
   # Hoặc tắt argument config:
   # Sửa trong launch file:
   # arguments=[],  # Bỏ '-d' argument


❌ LỖI 6: "Global planner failed"
   Nguyên nhân:
   - Goal nằm ngoài map
   - Goal trong vùng occupied
   - Không có đường đi
   
   Giải pháp:
   # Set goal trong vùng free (trắng) trên map
   # Không set goal quá xa
   # Kiểm tra costmap:
   ros2 topic echo /global_costmap/costmap


❌ LỖI 7: "Local planner oscillating"
   Nguyên nhân:
   - Parameters DWB sai
   - Footprint không khớp robot
   - Costmap inflation quá lớn
   
   Giải pháp:
   # Kiểm tra footprint trong nav2_params.yaml
   # Phải khớp với kích thước robot thật
   
   # Kiểm tra local costmap:
   ros2 topic echo /local_costmap/costmap

═══════════════════════════════════════════════════════════════════════════════
KIỂM TRA VÀ XÁC NHẬN
═══════════════════════════════════════════════════════════════════════════════

✅ CHECKLIST:

□ 1. Đã có bản đồ:
     ls -l ~/mecanum_robot_ws/src/mecanum_robot_bringup/maps/
     # Phải có: my_map.yaml và my_map.pgm

□ 2. Đã cấu hình nav2_params.yaml:
     cat ~/mecanum_robot_ws/.../config/nav2_params.yaml
     # Kiểm tra footprint, max velocities, v.v.

□ 3. Robot bringup chạy:
     ros2 topic list | grep -E "scan|odom|cmd_vel"

□ 4. TF tree đầy đủ:
     ros2 run tf2_tools view_frames
     # Phải có: map → odom → base_link

□ 5. Launch navigation:
     ros2 launch mecanum_robot_bringup navigation.launch.py

□ 6. Set initial pose trong RViz:
     # Click "2D Pose Estimate"

□ 7. Set navigation goal:
     # Click "2D Nav Goal"

□ 8. Robot di chuyển đến đích:
     ros2 topic echo /cmd_vel
     # Phải thấy velocity commands

═══════════════════════════════════════════════════════════════════════════════
THÔNG TIN THÊM
═══════════════════════════════════════════════════════════════════════════════

📖 Nav2 Documentation:
    https://navigation.ros.org/

🔧 Tuning Guide:
    https://navigation.ros.org/tuning/index.html

📊 Topics quan trọng:
    /map                 - Bản đồ từ map_server
    /scan_merged         - Laser scan
    /cmd_vel             - Velocity commands
    /odom                - Odometry
    /plan                - Global path
    /local_plan          - Local trajectory
    /global_costmap/*    - Global costmap
    /local_costmap/*     - Local costmap

⚡ Performance tips:
    - update_frequency: Cao = chính xác hơn nhưng tốn CPU
    - costmap size: Nhỏ = nhanh hơn nhưng nhìn gần hơn
    - planner_frequency: 1-2Hz là đủ cho hầu hết robot

═══════════════════════════════════════════════════════════════════════════════
"""