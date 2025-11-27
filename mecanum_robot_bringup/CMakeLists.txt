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
# Cài đặt các script Python
# ========================================================================

install(PROGRAMS
  scripts/velocity_bridge.py
  scripts/mecanum_odom_real.py
  scripts/hwt901b_driver.py
  scripts/laser_scan_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()