from setuptools import find_packages, setup

package_name = 'omni_bot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/launch',
         ['launch/odometry_test.launch.py']),
        ('share/' + package_name + '/launch',
         ['launch/kinematics_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soumyajit',
    maintainer_email='soumyajit@vssut.ac.in',
    description='This package intends to setup odometry and kinematics for omni_bot.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_node = omni_bot_bringup.odometry_node:main',
            'kinematics_node = omni_bot_bringup.kinematics_node:main',
        ],
    },
)
