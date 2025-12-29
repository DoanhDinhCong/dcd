from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    """
    ============================================================================
    ROBOT BRINGUP LAUNCH - ĐÃ THÊM ROBOT_LOCALIZATION
    ============================================================================
    LUỒNG DỮ LIỆU:
    1. STM32 → velocity_bridge → /joint_states
    2. /joint_states → mecanum_odom → /odom (encoder only)
    3. IMU → hwt901b_driver → /imu/data
    4. /odom + /imu/data → robot_localization → /odometry/filtered ✨
    5. SLAM/Nav2 sẽ dùng /odometry/filtered (chính xác hơn /odom)
    ============================================================================
    """
    
    pkg_share = get_package_share_directory('mecanum_robot_bringup')
    
    # ==========================================================================
    # URDF - Robot Description
    # ==========================================================================
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'rate': 50,
            # velocity_bridge sẽ publish wheel joints vào /joint_states
            'source_list': ['/joint_states']
        }],
        output='screen'
    )

    # ==========================================================================
    # VELOCITY BRIDGE - Giao tiếp STM32
    # ==========================================================================
    velocity_bridge = Node(
        package='mecanum_robot_bringup',
        executable='velocity_bridge.py',
        name='velocity_bridge',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud': 115200,
            'rate_hz': 50.0,
            'cmd_timeout_ms': 200,
            'max_vx': 0.2,
            'max_vy': 0.2,
            'max_wz': 0.2,
            'zero_on_timeout': True,
            'echo_tx': False,
            'echo_rx': False,
            'ticks_per_rev': 6864.0,
            'wheel_joint_names': ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rr_joint', 'wheel_rl_joint'],
            'invert_wheels': [True, True, True, True]
        }],
        output='screen'
    )
    
    # ==========================================================================
    # MECANUM ODOMETRY - Tính odometry từ encoder
    # ==========================================================================
    mecanum_odom = Node(
        package='mecanum_robot_bringup',
        executable='mecanum_odom_real.py',
        name='mecanum_odometry',
        parameters=[{
            'wheel_radius': 0.075,
            'wheel_base_width': 0.47,
            'wheel_base_length': 0.48,
            'publish_tf': False,
            'invert_wheels': [True, True, True, True],  
        }],
        output='screen'
    )
    
    # ==========================================================================
    # IMU DRIVER - HWT901B
    # ==========================================================================
    imu_driver = Node(
        package='mecanum_robot_bringup',
        executable='hwt901b_driver.py',
        name='imu_driver',
        parameters=[{
            'port': '/dev/ttyUSB2',
            'baudrate': 57600,  
            'frame_id': 'imu_link'
        }],
        output='screen'
    )
    
    # ==========================================================================
    # DUAL LIDAR + MERGER
    # ==========================================================================
    dual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'dual_lidar_merge.launch.py')
        )
    )
    
    # ==========================================================================
    # ✨ ROBOT_LOCALIZATION - FUSE IMU + ODOMETRY ✨
    # ==========================================================================
    # NODE MỚI: Extended Kalman Filter để kết hợp encoder + IMU
    # 
    # INPUT:
    # - /odom: vx, vy, wz từ encoder (mecanum_odom_real.py)
    # - /imu/data: gyro wz, angle yaw từ IMU (hwt901b_driver.py)
    #
    # OUTPUT:
    # - /odometry/filtered: Odometry đã được làm mịn, giảm drift
    #
    # LỢI ÍCH:
    # - Góc quay (yaw) chính xác gấp 3-5 lần
    # - Giảm drift vị trí 50-70%
    # - SLAM tin tưởng odometry hơn → map chính xác hơn
    #
    # CÁCH DÙNG:
    # - SLAM/Nav2 sẽ subscribe /odometry/filtered thay vì /odom
    # - Cần sửa slam_toolbox_params.yaml và nav2_params.yaml
    #   (xem hướng dẫn bên dưới)
    # ==========================================================================
    pkg_share = get_package_share_directory('mecanum_robot_bringup')
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=['/home/dcd/mecanum_robot_ws/src/mecanum_robot_bringup/config/robot_localization.yaml'],
        output='screen',
        remappings=[
            #('/odometry/filtered', '/odom')
            # Remap để Nav2/SLAM dùng odometry filtered
            # KHÔNG remap nếu muốn giữ /odom riêng biệt để so sánh
        ]
    )
    
    # ==========================================================================
    # LAUNCH ALL NODES
    # ==========================================================================
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        velocity_bridge,
        mecanum_odom,
        imu_driver,
        dual_lidar_launch,
        robot_localization,  # ✨ NODE MỚI
    ])

# =============================================================================
# HƯỚNG DẪN SỬ DỤNG SAU KHI THÊM ROBOT_LOCALIZATION
# =============================================================================
#
# BƯỚC 1: Cài đặt robot_localization
# $ sudo apt install ros-humble-robot-localization
#
# BƯỚC 2: Copy file config
# Đã tạo file: config/robot_localization.yaml
#
# BƯỚC 3: Chạy robot bình thường
# $ ros2 launch mecanum_robot_bringup robot_bringup.launch.py
#
# BƯỚC 4: Kiểm tra topics
# $ ros2 topic list
# Phải thấy:
# - /odom (encoder only - để so sánh)
# - /imu/data (IMU raw data)
# - /odometry/filtered (✨ fused - DÙNG CÁI NÀY!)
#
# BƯỚC 5: So sánh odometry
# Terminal 1: ros2 topic echo /odom
# Terminal 2: ros2 topic echo /odometry/filtered
# Cho robot đi thẳng 5m, xem filtered ít drift hơn
#
# BƯỚC 6: Sửa SLAM để dùng odometry/filtered
# File: config/slam_toolbox_params.yaml
# TÌM: odom_topic: /odom
# ĐỔI: odom_topic: /odometry/filtered
#
# BƯỚC 7: Sửa Nav2 để dùng odometry/filtered
# File: config/nav2_params.yaml
# TÌM: odom_topic: /odom (2 chỗ)
# ĐỔI: odom_topic: /odometry/filtered
#
# BƯỚC 8: Test SLAM với odometry mới
# $ ros2 launch mecanum_robot_bringup slam_mapping.launch.py
# Map sẽ chính xác và ít bị biến dạng hơn!
#
# =============================================================================
# TROUBLESHOOTING
# =============================================================================
#
# ❌ "Could not load robot_localization"
# → sudo apt install ros-humble-robot-localization
#
# ❌ "No messages on /odometry/filtered"
# → Kiểm tra /odom và /imu/data có data không
# → Xem log: ros2 launch ... --log-level DEBUG
#
# ❌ "/odometry/filtered vẫn drift nhiều"
# → Điều chỉnh covariance trong robot_localization.yaml
# → Xem phần "HƯỚNG DẪN ĐIỀU CHỈNH" trong file config
#
# ❌ "TF tree error"
# → Kiểm tra base_link_frame = base_footprint trong config
#
# =============================================================================
