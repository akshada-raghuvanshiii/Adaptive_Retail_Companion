import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arc_ui_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', [f for f in glob('web/*') if os.path.isfile(f)]),
        ('share/' + package_name + '/web/fonts', glob('web/fonts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshada-raghuvanshi',
    maintainer_email='akshada-raghuvanshi@todo.todo',
    description='ARC shopping UI connected to the ARC stack',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['ui_bridge = arc_ui_bridge.server:main']},
)
