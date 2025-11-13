# setup.py (cùng cấp với package.xml)
from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Config files  
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
        # RViz configs (nếu có)
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hoang Viet',
    maintainer_email='vtran13102000@gmail.com',
    description='Mecanum robot bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_bridge.py = mecanum_robot_bringup.velocity_bridge:main',
            'mecanum_odom_real.py = mecanum_robot_bringup.mecanum_odom_real:main',
            'laser_scan_merger.py = mecanum_robot_bringup.laser_scan_merger:main',
            'hwt901b_driver.py = mecanum_robot_bringup.hwt901b_driver:main',
            'test_odometry_comparison.py = mecanum_robot_bringup.test_odometry_comparison:main',
        ],
    },
)
