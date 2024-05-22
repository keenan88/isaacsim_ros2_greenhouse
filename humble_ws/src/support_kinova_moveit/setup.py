from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'support_kinova_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))

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
            'joint_forwarder = support_kinova_moveit.joint_forwarder:main',
            'joint_command_forwarder = support_kinova_moveit.joint_command_forwarder:main',
        ],
    },
)
