from setuptools import setup
package_name = "control"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/pid_gains.yaml"]),
    ],
    install_requires=["setuptools", "simple-pid"],
    zip_safe=True,
    maintainer="Njord",
    maintainer_email="njord@stud.ntnu.no",
    description="Control nodes for the Njord 2026 USV",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nav_to_pid = control.nav_to_pid:main",
            "pid_controller = control.pid_controller:main",
            "actuator_driver = control.actuator_driver:main",
            "control_all = control.launch_all:main",
            "pico_bridge = control.pico_bridge:main",
        ],
    },
)
