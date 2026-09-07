import os
from glob import glob

from setuptools import find_packages, setup

package_name = "asket_sim"

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
    description="Simulated data sources for the whole Asket stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = asket_sim.sim_node:main",
            "fake_sonar_node = asket_sim.fake_sonar_node:main",
            "make_sonar_raw = asket_sim.make_sonar_raw:main",
        ],
    },
)
