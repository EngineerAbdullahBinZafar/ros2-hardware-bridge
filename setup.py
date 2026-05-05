from setuptools import setup
import os

package_name = 'ros2_bridge'

setup(
    name='ros2_hardware_bridge',
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/ros2_hardware_bridge', ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='EngineerAbdullahBinZafar',
    maintainer_email='abz.king.1.9.2003@gmail.com',
    description='Plug-and-play serial bridge for ROS2 to embedded hardware.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = ros2_bridge.bridge_node:main',
        ],
    },
)
