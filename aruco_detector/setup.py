import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andria Nicholas',
    maintainer_email='maryjandria@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"aruco_marker_action.py = {package_name}.aruco_marker_action.py:main",
            f"aruco = {package_name}.aruco:main",
            f"aruco_detector = {package_name}.aruco_detector:main",
            f"aruco_generator = {package_name}.aruco_generator:main",
            f"navigate = {package_name}.navigate:main",
            f"dwa_navigator = {package_name}.dwa_navigator:main"
        ],
    },
)
