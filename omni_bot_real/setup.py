from setuptools import find_packages, setup

package_name = 'omni_bot_real'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/real.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soumyajit',
    maintainer_email='soumyajit@vssut.ac.in',
    description='A bringup package for whole robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
