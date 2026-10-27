from glob import glob
from setuptools import find_packages, setup

PACKAGE = 'tb4_gesture_control'
setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/models', glob('models/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hung',
    maintainer_email='hung@example.com',
    description='Confirmed, interruptible gesture control for TurtleBot 4',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'gesture_recognition_node = tb4_gesture_control.gesture_recognition_node:main',
        'robot_behavior_node = tb4_gesture_control.robot_behavior_node:main',
    ]},
)
