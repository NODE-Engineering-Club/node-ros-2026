from setuptools import setup

package_name = "mission_maneuvering_pathfinding"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/maneuvering_pathfinding_mission.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Njord",
    maintainer_email="njord@stud.ntnu.no",
    description="Task 9.1 (Maneuvering and Path Finding) mission sequencer for the Njord 2026 USV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "maneuvering_pathfinding_mission = "
            "mission_maneuvering_pathfinding.maneuvering_pathfinding_mission:main",
        ],
    },
)
