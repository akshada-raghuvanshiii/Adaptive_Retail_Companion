from glob import glob

from setuptools import setup

package_name = "arc_store_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/scenarios", glob("scenarios/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ARC team",
    maintainer_email="todo@example.com",
    description="Supermarket sandbox world for ARC (Gazebo Harmonic, ROS 2 Jazzy)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_recorder = arc_store_sim.camera_recorder:main",
        ],
    },
)
