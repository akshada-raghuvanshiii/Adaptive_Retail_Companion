from setuptools import find_packages, setup

package_name = 'arc_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshada-raghuvanshi',
    maintainer_email='akshada-raghuvanshi@todo.todo',
    description='ARC global path planner',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dijkstra_planner = arc_planner.dijkstra_planner:main',
	    'reactive_avoider = arc_planner.reactive_avoider:main',
	    'path_follower = arc_planner.path_follower:main',
	    'planned_path_to_teb = arc_planner.planned_path_to_teb:main',
	    'shopping_manager = arc_planner.shopping_manager:main',
        ],
    },
)
