#!/usr/bin/env python3
# =============================================================================
# MECANUM ODOMETRY - Tính vị trí robot từ encoder
# =============================================================================
# CHỨC NĂNG:
# - Nhận vận tốc bánh từ JointState
# - Tính vận tốc robot (vx, vy, wz) theo kinematic mecanum
# - Tích phân để tính vị trí (x, y, theta)
# - Publish Odometry message và TF transform
# =============================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
import math

class MecanumOdometryReal(Node):
    """
    Node tính odometry cho robot mecanum 4 bánh
    
    KINEMATIC FORMULA (mecanum wheel):
    Vx_robot = R/4 * (v_FL + v_FR + v_RR + v_RL)
    Vy_robot = R/4 * (-v_FL + v_FR - v_RR + v_RL)
    Wz = R/(4*(Lx+Ly)) * (-v_FL + v_FR + v_RR - v_RL)
    
    Trong đó:
    - R: Bán kính bánh
    - Lx: Nửa chiều dài robot (trước-sau)
    - Ly: Nửa chiều rộng robot (trái-phải)
    - v_XX: Vận tốc góc bánh (rad/s)
    
    THAM SỐ CẦN ĐIỀU CHỈNH:
    - wheel_radius: Đo đường kính bánh thật, chia 2
    - wheel_base_length: Đo khoảng cách tâm bánh trước-sau
    - wheel_base_width: Đo khoảng cách tâm bánh trái-phải
    - invert_wheels: Phải giống với velocity_bridge.py
    """
    
    def __init__(self):
        super().__init__('mecanum_odometry_real')
        
        # =====================================================================
        # ROBOT DIMENSIONS
        # =====================================================================
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_base_width', 0.47)
        self.declare_parameter('wheel_base_length', 0.48)
        self.declare_parameter('publish_tf', False)
        
        # =====================================================================
        # WHEEL JOINT NAMES (PHẢI GIỐNG VỚI URDF VÀ velocity_bridge.py)
        # =====================================================================
        self.declare_parameter('invert_wheels', [True, True, True, True])
        
        self.joint_names = [
            'wheel_fl_joint',
            'wheel_fr_joint',
            'wheel_rr_joint',
            'wheel_rl_joint'
        ]
        self.wheel_radius = self.get_parameter('wheel_radius').value
        lx = self.get_parameter('wheel_base_length').value / 2.0
        ly = self.get_parameter('wheel_base_width').value / 2.0
        self.lx_ly = lx + ly
        self.publish_tf = self.get_parameter('publish_tf').value
        self.invert = self.get_parameter('invert_wheels').value
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriber
        self.js_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10)
        
        self.get_logger().info('✅ Mecanum Odometry Started')
        self.get_logger().info(f'   - Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'   - Base: {self.get_parameter("wheel_base_length").value}m x {self.get_parameter("wheel_base_width").value}m')
        self.get_logger().info(f'   - Invert wheels: {self.invert}')
    
    def joint_states_callback(self, msg: JointState):
        """Callback nhận vận tốc bánh từ velocity_bridge"""
        
        try:
            # Tìm index của từng joint trong message
            fl_idx = msg.name.index(self.joint_names[0])
            fr_idx = msg.name.index(self.joint_names[1])
            rr_idx = msg.name.index(self.joint_names[2])
            rl_idx = msg.name.index(self.joint_names[3])
        except ValueError as e:
            self.get_logger().warn(f'Joint not found in message: {e}', 
                                  throttle_duration_sec=5.0)
            return
        
        # Kiểm tra có đủ velocity data
        if len(msg.velocity) <= max(fl_idx, fr_idx, rr_idx, rl_idx):
            self.get_logger().warn('Insufficient velocity data', 
                                  throttle_duration_sec=5.0)
            return

        
        # =====================================================================
        # LẤY VẬN TỐC BÁNH
        # =====================================================================
        v_fl = msg.velocity[fl_idx] 
        v_fr = msg.velocity[fr_idx]
        v_rr = msg.velocity[rr_idx]
        v_rl = msg.velocity[rl_idx]

        # =====================================================================
        # MECANUM KINEMATIC FORWARD
        # =====================================================================
        r = self.wheel_radius
        
        # Công thức kinematic mecanum wheel
        vx_robot = r / 4.0 * (v_fl + v_fr + v_rr + v_rl)
        vy_robot = r / 4.0 * (-v_fl + v_fr - v_rr + v_rl)
        wz = r / (4.0 * self.lx_ly) * (-v_fl + v_fr + v_rr - v_rl)
        
        # =====================================================================
        # INTEGRATION - Tích phân vận tốc thành vị trí
        # =====================================================================
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Kiểm tra dt hợp lệ
        if dt <= 0.0 or dt > 1.0:
            self.last_time = current_time
            return
        
        # Chuyển vận tốc từ robot frame sang odom frame
        delta_x = (vx_robot * math.cos(self.theta) - vy_robot * math.sin(self.theta)) * dt
        delta_y = (vx_robot * math.sin(self.theta) + vy_robot * math.cos(self.theta)) * dt
        delta_theta = wz * dt
        
        # Cập nhật vị trí
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta về [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Quaternion từ yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        # =====================================================================
        # PUBLISH ODOMETRY AND TF
        # =====================================================================
        
        # Publish TF transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'  # ✅ ĐÃ SỬA
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'  # ✅ ĐÃ SỬA: Thống nhất với TF
        
        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Twist (vận tốc)
        odom.twist.twist.linear.x = vx_robot
        odom.twist.twist.linear.y = vy_robot
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz
        
        # =====================================================================
        # COVARIANCE - ĐIỀU CHỈNH CHO BÁNH MECANUM
        # =====================================================================
        
        # Pose covariance [x, y, z, rot_x, rot_y, rot_z]
        # Chỉ set x(0), y(7), theta(35)
        odom.pose.covariance[0] = 0.1   # ✅ x variance (10cm std)
        odom.pose.covariance[7] = 0.1   # ✅ y variance (10cm std)
        odom.pose.covariance[35] = 0.2  # ✅ theta variance (~25 degree std)
        
        # Twist covariance
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[7] = 0.1
        odom.twist.covariance[35] = 0.2
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()