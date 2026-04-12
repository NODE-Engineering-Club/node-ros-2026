from setuptools import setup

package_name = "sensors"

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
    description="Sensor drivers for the Njord 2026 USV",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_driver = sensors.camera_driver:main",
            "lidar_driver = sensors.lidar_driver:main",
            "imu_gps_driver = sensors.imu_gps_driver:main",
            "sensors_all = sensors.launch_all:main",
        ],
    },
)
