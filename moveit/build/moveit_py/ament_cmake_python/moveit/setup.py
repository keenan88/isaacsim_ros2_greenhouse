from setuptools import find_packages
from setuptools import setup

setup(
    name='moveit',
    version='2.9.0',
    packages=find_packages(
        include=('moveit', 'moveit.*')),
)
