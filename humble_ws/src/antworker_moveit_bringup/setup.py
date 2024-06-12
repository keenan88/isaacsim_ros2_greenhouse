from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'antworker_moveit_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ee_point_server = antworker_moveit_bringup.ee_point_server:main',
            'trajectory_server = antworker_moveit_bringup.trajectory_server:main',
            'trajectory_server_sim = antworker_moveit_bringup.trajectory_server_sim:main',
            'joint_command_forwarder = antworker_moveit_bringup.joint_command_forwarder:main',
            'joint_state_forwarder = antworker_moveit_bringup.joint_state_forwarder:main',
            'dummy_wheel_joint_publisher = antworker_moveit_bringup.dummy_wheel_joint_publisher:main'
        ],
    },
)
