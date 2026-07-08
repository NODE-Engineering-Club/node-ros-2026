from setuptools import setup

package_name = "calibration"

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
    description="Camera–LiDAR extrinsic calibration tooling for the Njord 2026 USV",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_to_cloud         = calibration.scan_to_cloud:main",
            "collect_data          = calibration.collect_data:main",
            "calibrate             = calibration.calibrate:main",
            "extrinsic_tf_publisher = calibration.extrinsic_tf_publisher:main",
        ],
    },
)
