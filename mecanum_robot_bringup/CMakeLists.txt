cmake_minimum_required(VERSION 3.8)
project(mecanum_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Cài đặt thư mục resources
install(DIRECTORY
  launch
  config
  urdf
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# ========================================================================
# ✅ SỬA LỖI MAJOR #9: XÓA encoder_joint_publisher.py
# ========================================================================
# LÝ DO: File này TRÙNG chức năng với velocity_bridge.py
#        - Cả 2 đều đọc encoder từ STM32
#        - Cả 2 đều publish JointState
#        - Chạy cả 2 sẽ gây conflict và tốn CPU
# 
# GIẢI PHÁP: Chỉ dùng velocity_bridge.py vì nó đầy đủ hơn:
#            - Có cả TX (gửi cmd_vel) và RX (đọc encoder)
#            - Có watchdog timeout
#            - Có log control
# ========================================================================

install(PROGRAMS
  scripts/velocity_bridge.py
  scripts/mecanum_odom_real.py
  scripts/hwt901b_driver.py
  scripts/laser_scan_merger.py
  scripts/test_odometry_comparison.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
