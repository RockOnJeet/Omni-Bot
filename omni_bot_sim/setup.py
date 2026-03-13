from setuptools import find_packages, setup
import os

# Modify if your package name is different
package_name = 'omni_bot_sim'

package_dir = os.path.dirname(__file__)

# folders under this package to include in data_files (relative to this package dir)
selected_folders = ['config', 'launch', 'worlds']

# start with the existing static entries
data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Walk each selected folder and add files preserving relative subdirectories
for folder in selected_folders:
    src_folder = os.path.join(package_dir, folder)
    if os.path.exists(src_folder):
        for dirpath, dirnames, filenames in os.walk(src_folder):
            if not filenames:
                continue
            # relative directory inside the package (e.g. 'description/models')
            rel_dir = os.path.relpath(dirpath, package_dir)
            dest_dir = os.path.join(
                'share', package_name, rel_dir).replace(os.path.sep, '/')
            file_paths = [os.path.join(rel_dir, f).replace(
                os.path.sep, '/') for f in filenames]
            data_files.append((dest_dir, file_paths))
    else:
        # Folder does not exist; error out
        raise FileNotFoundError(
            f"Required folder '{folder}' not found in package '{package_name}'")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soumyajit',
    maintainer_email='soumyajit@vssut.ac.in',
    description='This package contains the simulation setup for omni_bot.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_vel_pub = omni_bot_sim.wheel_vel_pub:main',
        ],
    },
)
