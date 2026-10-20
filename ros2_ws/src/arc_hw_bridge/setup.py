from glob import glob

from setuptools import find_packages, setup

package_name = 'arc_hw_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/firmware/arc_cart_motor', glob('firmware/arc_cart_motor/*.ino')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshada-raghuvanshi',
    maintainer_email='akshada-raghuvanshi@todo.todo',
    description='ARC motor bridge: /cmd_vel to the Arduino Uno and real wheels',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'motor_bridge = arc_hw_bridge.bridge:main',
            'motor_bench = arc_hw_bridge.bench:main',
        ],
    },
)
