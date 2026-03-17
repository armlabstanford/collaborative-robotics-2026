from setuptools import setup
import os
from glob import glob

package_name = 'tidybot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Qiu and Joonwon Kang',
    maintainer_email='aqiu34@stanford.edu, jwkang@stanford.edu',
    description='Control nodes for TidyBot2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_controller_node = tidybot_control.arm_controller_node:main',
            'arm_wrapper_node = tidybot_control.arm_wrapper_node:main',
            # 'interbotix_arm_node = tidybot_control.interbotix_arm_node:main', # Deprecated, may be used in future to get rid of interbotix dependencies
            'phoenix6_base_node = tidybot_control.phoenix6_base_node:main',
            'pan_tilt_node = tidybot_control.pan_tilt_node:main',
            'dynamixel_bus_node = tidybot_control.dynamixel_bus_node:main',
            'gripper_wrapper_node = tidybot_control.gripper_wrapper_node:main',
            'microphone_node = tidybot_control.microphone_node:main',
            'phone_teleop_server = tidybot_control.phone_teleop_server:main',
        ],
    },
)
