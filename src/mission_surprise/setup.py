from setuptools import setup

package_name = "mission_surprise"

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
            ["launch/surprise_mission.launch.py"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/surprise_9_4_waypoints.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Njord",
    maintainer_email="njord@stud.ntnu.no",
    description="Task 9.4 (Surprise) mission sequencer for the Njord 2026 USV",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "surprise_mission = mission_surprise.surprise_mission:main",
        ],
    },
)
