from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'goal_pose_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'video'), glob('video/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usama',
    maintainer_email='usamasattar.3347@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmark_publisher = goal_pose_publisher.landmark_publisher:main',
            'goal_pose_publisher = goal_pose_publisher.goal_pose_publisher:main',
        ],
    },
)
