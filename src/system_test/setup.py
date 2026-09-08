import os
from glob import glob

from setuptools import find_packages, setup

package_name = "system_test"

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
    description="Built-in test: passive checks, GO/NO-GO verdict.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_test_node = system_test.system_test_node:main",
        ],
    },
)
