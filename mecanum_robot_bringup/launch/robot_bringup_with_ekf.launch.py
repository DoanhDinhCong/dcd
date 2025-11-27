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
    ROBOT BRINGUP LAUNCH 
    ============================================================================
    LUỒNG DỮ LIỆU:
    1. STM32 → velocity_bridge → /joint_states
    2. /joint_states → mecanum_odom → /odom (encoder only)
    3. IMU → hwt901b_driver → /imu/data
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
    # ✅ ĐÃ THÊM: invert_wheels parameter
    # ==========================================================================
    mecanum_odom = Node(
        package='mecanum_robot_bringup',
        executable='mecanum_odom_real.py',
        name='mecanum_odometry',
        parameters=[{
            'wheel_radius': 0.075,
            'wheel_base_width': 0.47,
            'wheel_base_length': 0.48,
            'publish_tf': True,
            'invert_wheels': [True, True, True, True],  # ✅ ĐÃ THÊM
        }],
        output='screen'
    )
    
    # ==========================================================================
    # IMU DRIVER - HWT901B
    # ==========================================================================
    # ✅ ĐÃ SỬA: baudrate 115200 → 9600 (chuẩn HWT901B)
    # ==========================================================================
    imu_driver = Node(
        package='mecanum_robot_bringup',
        executable='hwt901b_driver.py',
        name='imu_driver',
        parameters=[{
            'port': '/dev/ttyUSB2',
            'baudrate': 9600,  # ✅ ĐÃ SỬA: 115200 → 9600
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
    # LAUNCH ALL NODES
    # ==========================================================================
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        velocity_bridge,
        mecanum_odom,
        imu_driver,
        dual_lidar_launch,
        
    ])

