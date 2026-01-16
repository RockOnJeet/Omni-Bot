from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'omni_bot_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soumyajit',
    maintainer_email='soumyajit@vssut.ac.in',
    description='Base controller for omni-directional robot - converts cmd_vel to wheel velocities',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_controller = omni_bot_base.base_controller:main',
        ],
    },
)
