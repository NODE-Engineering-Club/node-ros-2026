from setuptools import find_packages, setup


package_name = "competition_manager"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Heleri Koltsin",
    maintainer_email="heleri.koltsin@students.iaac.net",
    description=(
        "High-level competition task and lifecycle coordination "
        "for the Njord 2026 USV."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "competition_manager = "
                "competition_manager.competition_manager:main"
            ),
        ],
    },
)
