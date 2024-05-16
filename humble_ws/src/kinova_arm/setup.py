from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'kinova_arm'



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'description/urdf'), glob(os.path.join('description/urdf', 'GEN3_6DOF_VISION_URDF_ARM_V01.urdf'))),
        (os.path.join('share', package_name, 'description/meshes'), glob(os.path.join('description/meshes', '*'))),
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
            'joint_state_forwarder = kinova_arm.joint_state_forwarder:main',
        ],
    },
)
