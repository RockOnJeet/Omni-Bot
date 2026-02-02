from setuptools import find_packages, setup

package_name = 'omni_bot_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odometry_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soumyajit',
    maintainer_email='soumyajit@vssut.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odometry_node = omni_bot_odometry.odometry_node:main',
        ],
    },
)
