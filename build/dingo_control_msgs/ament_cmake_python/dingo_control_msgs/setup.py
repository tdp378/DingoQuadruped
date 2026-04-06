from setuptools import find_packages
from setuptools import setup

setup(
    name='dingo_control_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('dingo_control_msgs', 'dingo_control_msgs.*')),
)
