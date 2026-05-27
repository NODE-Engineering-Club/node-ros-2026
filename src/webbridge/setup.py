from setuptools import setup

package_name = "webbridge"

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
    description="HTTP bridge exposing ROS topics to the Njord web dashboard",
    license="MIT",
    entry_points={
        "console_scripts": [
            "webbridge_node = webbridge.webbridge_node:main",
        ],
    },
)
