from setuptools import setup

package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Njord",
    maintainer_email="njord@stud.ntnu.no",
    description="Perception nodes for the Njord 2026 USV",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lidar_obstacle_node = perception.lidar_obstacle_node:main",
            "fusion_node = perception.fusion_node:main",
            "buoy_tracker_node = perception.buoy_tracker_node:main",
            "perception_all = perception.launch_all:main",
        ],
    },
)
