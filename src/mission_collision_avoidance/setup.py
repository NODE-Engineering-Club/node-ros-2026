from setuptools import setup

package_name = "mission_collision_avoidance"

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
            ["launch/collision_avoidance_mission.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Njord",
    maintainer_email="njord@stud.ntnu.no",
    description="Task 9.2 (Collision Avoidance) mission sequencer for the Njord 2026 USV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "collision_avoidance_mission = "
            "mission_collision_avoidance.collision_avoidance_mission:main",
        ],
    },
)
