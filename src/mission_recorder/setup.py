import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mission_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NODE Engineering Club",
    maintainer_email="node@example.org",
    description="Mission recording, mission files and export.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_recorder_node = mission_recorder.mission_recorder_node:main",
            "merge_svlog = mission_recorder.merge_svlog:main",
        ],
    },
)
