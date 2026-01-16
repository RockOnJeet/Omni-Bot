from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'omni_bot_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omni-Bot Developer',
    maintainer_email='your_email@example.com',
    description='Encoder-based odometry for omnidirectional robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = omni_bot_odometry.odometry_node:main',
        ],
    },
)
