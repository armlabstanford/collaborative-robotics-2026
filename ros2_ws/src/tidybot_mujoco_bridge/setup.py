from setuptools import setup
import os
from glob import glob

package_name = 'tidybot_mujoco_bridge'

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
    maintainer='Alex Qiu and Matt Strong',
    maintainer_email='aqiu34@stanford.edu, mastro1@stanford.edu',
    description='MuJoCo simulation bridge for TidyBot2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mujoco_bridge_node = tidybot_mujoco_bridge.mujoco_bridge_node:main',
        ],
    },
)
