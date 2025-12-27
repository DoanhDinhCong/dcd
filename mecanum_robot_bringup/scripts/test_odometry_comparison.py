#!/usr/bin/env python3
# =============================================================================
# SCRIPT TEST SO SÁNH ODOMETRY
# =============================================================================
# Chức năng: So sánh /odom (encoder only) vs /odometry/filtered (encoder+IMU)
# Cách dùng: ros2 run mecanum_robot_bringup test_odometry_comparison.py
# =============================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class OdometryComparisonTest(Node):
    """
    Node để test và so sánh 2 loại odometry:
    1. /odom - chỉ từ encoder (drift nhiều)
    2. /odometry/filtered - từ encoder + IMU (chính xác hơn)
    
    CÁCH SỬ DỤNG:
    1. Chạy script này
    2. Đặt robot tại vị trí ban đầu
    3. Nhấn ENTER để bắt đầu test
    4. Điều khiển robot (teleop) đi 1 vòng quanh phòng
    5. Quay lại vị trí ban đầu
    6. Nhấn ENTER để xem kết quả

    
    """
    
    def __init__(self):
        super().__init__('odometry_comparison_test')
        
        # Subscribe 2 odometry topics
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.filtered_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_callback, 10)
        
        # Lưu vị trí ban đầu
        self.odom_start = None
        self.filtered_start = None
        
        # Lưu vị trí hiện tại
        self.odom_current = None
        self.filtered_current = None
        
        # Flag test
        self.testing = False
        
        self.get_logger().info('✅ Odometry Comparison Test Started!')
        self.get_logger().info('   Waiting for odometry data...')
    
    def odom_callback(self, msg):
        """Callback cho /odom (encoder only)"""
        self.odom_current = msg
        
        # Lưu vị trí ban đầu khi bắt đầu test
        if self.testing and self.odom_start is None:
            self.odom_start = msg
            self.get_logger().info('📍 Recorded /odom start position')
    
    def filtered_callback(self, msg):
        """Callback cho /odometry/filtered (encoder + IMU)"""
        self.filtered_current = msg
        
        # Lưu vị trí ban đầu khi bắt đầu test
        if self.testing and self.filtered_start is None:
            self.filtered_start = msg
            self.get_logger().info('📍 Recorded /odometry/filtered start position')
    
    def calculate_error(self, start_odom, end_odom):
        """Tính sai số vị trí giữa 2 điểm"""
        if start_odom is None or end_odom is None:
            return None, None
        
        # Lấy vị trí x, y
        x0 = start_odom.pose.pose.position.x
        y0 = start_odom.pose.pose.position.y
        x1 = end_odom.pose.pose.position.x
        y1 = end_odom.pose.pose.position.y
        
        # Tính khoảng cách Euclid
        distance_error = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        
        # Tính sai số góc yaw
        # Chuyển quaternion thành yaw
        def quat_to_yaw(q):
            return math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        
        yaw0 = quat_to_yaw(start_odom.pose.pose.orientation)
        yaw1 = quat_to_yaw(end_odom.pose.pose.orientation)
        
        # Normalize góc về [-pi, pi]
        yaw_error = yaw1 - yaw0
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        yaw_error_deg = abs(math.degrees(yaw_error))
        
        return distance_error, yaw_error_deg
    
    def start_test(self):
        """Bắt đầu test"""
        self.get_logger().info('=' * 70)
        self.get_logger().info('🚀 STARTING ODOMETRY COMPARISON TEST')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('HƯỚNG DẪN:')
        self.get_logger().info('1. Đặt robot tại vị trí ban đầu (đánh dấu bằng băng dính)')
        self.get_logger().info('2. Dùng teleop để điều khiển robot:')
        self.get_logger().info('   - Đi 1 vòng quanh phòng')
        self.get_logger().info('   - HOẶC đi thẳng 10m, quay lại')
        self.get_logger().info('   - HOẶC quay tròn 10 vòng tại chỗ')
        self.get_logger().info('3. Quay về vị trí ban đầu')
        self.get_logger().info('4. Nhấn ENTER trong terminal này để xem kết quả')
        self.get_logger().info('')
        
        # Reset
        self.odom_start = None
        self.filtered_start = None
        self.testing = True
        
        # Chờ để record vị trí ban đầu
        time.sleep(1.0)
        
        if self.odom_start is None or self.filtered_start is None:
            self.get_logger().error('❌ Không nhận được dữ liệu odometry!')
            self.get_logger().error('   Kiểm tra: ros2 topic list')
            self.get_logger().error('   Phải có /odom và /odometry/filtered')
            return False
        
        self.get_logger().info('✅ Đã ghi nhận vị trí ban đầu!')
        self.get_logger().info('   Bắt đầu điều khiển robot...')
        self.get_logger().info('')
        
        return True
    
    def end_test(self):
        """Kết thúc test và hiển thị kết quả"""
        self.testing = False
        
        if self.odom_current is None or self.filtered_current is None:
            self.get_logger().error('❌ Không có dữ liệu odometry!')
            return
        
        # Tính sai số
        odom_dist_error, odom_yaw_error = self.calculate_error(
            self.odom_start, self.odom_current)
        filtered_dist_error, filtered_yaw_error = self.calculate_error(
            self.filtered_start, self.filtered_current)
        
        # Hiển thị kết quả
        self.get_logger().info('=' * 70)
        self.get_logger().info('📊 KẾT QUẢ SO SÁNH ODOMETRY')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        
        self.get_logger().info('🔴 /odom (ENCODER ONLY):')
        self.get_logger().info(f'   Sai số vị trí: {odom_dist_error*100:.2f} cm')
        self.get_logger().info(f'   Sai số góc: {odom_yaw_error:.2f} độ')
        self.get_logger().info('')
        
        self.get_logger().info('🟢 /odometry/filtered (ENCODER + IMU):')
        self.get_logger().info(f'   Sai số vị trí: {filtered_dist_error*100:.2f} cm')
        self.get_logger().info(f'   Sai số góc: {filtered_yaw_error:.2f} độ')
        self.get_logger().info('')
        
        # Tính phần trăm cải thiện
        if odom_dist_error > 0:
            improvement_dist = (1 - filtered_dist_error / odom_dist_error) * 100
            self.get_logger().info('💡 CẢI THIỆN:')
            self.get_logger().info(f'   Vị trí: {improvement_dist:.1f}% chính xác hơn')
        
        if odom_yaw_error > 0:
            improvement_yaw = (1 - filtered_yaw_error / odom_yaw_error) * 100
            self.get_logger().info(f'   Góc quay: {improvement_yaw:.1f}% chính xác hơn')
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        
        # Đánh giá
        self.get_logger().info('🎯 ĐÁNH GIÁ:')
        if improvement_dist > 30:
            self.get_logger().info('   ✅ Robot_localization hoạt động TỐT!')
            self.get_logger().info('   ✅ IMU đang giúp giảm drift hiệu quả!')
        elif improvement_dist > 10:
            self.get_logger().info('   ⚠️  Robot_localization hoạt động TRUNG BÌNH')
            self.get_logger().info('   💡 Có thể cần điều chỉnh covariance')
        else:
            self.get_logger().info('   ❌ Robot_localization không hiệu quả')
            self.get_logger().info('   💡 Kiểm tra:')
            self.get_logger().info('      - IMU có đang hoạt động?')
            self.get_logger().info('      - Covariance có đúng không?')
            self.get_logger().info('      - Frame_id có khớp không?')
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('')

def main():
    rclpy.init()
    node = OdometryComparisonTest()
    
    # Spin trong background thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Chờ cho topics sẵn sàng
        print('\n🔄 Đang chờ odometry topics...')
        time.sleep(2.0)
        
        while True:
            print('\n' + '='*70)
            print('ODOMETRY COMPARISON TEST')
            print('='*70)
            print('\nLựa chọn:')
            print('1. Bắt đầu test mới')
            print('2. Xem status hiện tại')
            print('3. Thoát')
            
            choice = input('\nNhập lựa chọn (1-3): ').strip()
            
            if choice == '1':
                # Bắt đầu test
                if node.start_test():
                    input('\n⏸️  Nhấn ENTER sau khi robot quay về vị trí ban đầu...')
                    node.end_test()
            
            elif choice == '2':
                # Hiển thị status
                print('\n📊 STATUS:')
                if node.odom_current:
                    print(f'   /odom: OK (x={node.odom_current.pose.pose.position.x:.2f}, '
                          f'y={node.odom_current.pose.pose.position.y:.2f})')
                else:
                    print('   /odom: ❌ Không có data')
                
                if node.filtered_current:
                    print(f'   /odometry/filtered: OK (x={node.filtered_current.pose.pose.position.x:.2f}, '
                          f'y={node.filtered_current.pose.pose.position.y:.2f})')
                else:
                    print('   /odometry/filtered: ❌ Không có data')
            
            elif choice == '3':
                print('\n👋 Tạm biệt!')
                break
            
            else:
                print('\n❌ Lựa chọn không hợp lệ!')
    
    except KeyboardInterrupt:
        print('\n\n👋 Đã dừng test!')
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()