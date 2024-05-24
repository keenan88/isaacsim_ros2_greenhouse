from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'antworker_description'

def get_data_files():
    data_files = []
    for root, dirs, files in os.walk('description'):
        for file in files:
            data_files.append(('share/' + package_name, [os.path.join(root, file)]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        #('share/' + package_name + '/GEN3_6DOF_VISION_URDF_ARM_V01.urdf', ['description/arm/urdf/GEN3_6DOF_VISION_URDF_ARM_V01.urdf'])
        #('share/' + package_name, glob('description/base/meshes/**')),

        (os.path.join('share', package_name, 'description/combined'), glob(os.path.join('description/combined', 'worker_with_arm.urdf'))),

        (os.path.join('share', package_name, 'description/arm/meshes'), glob(os.path.join('description/arm/meshes', '*'))),
        (os.path.join('share', package_name, 'description/base/meshes'), glob(os.path.join('description/base/meshes', '*'))),

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
        ],
    },
)
