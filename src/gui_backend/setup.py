import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gui_backend"

# The built frontend lands in gui_backend/static/ and is installed with the
# package: one process serves the API and the page, because there is no CDN in
# Namibia and no second server to run on the Jetson.
static_files = [
    (
        os.path.join("share", package_name, "static", os.path.relpath(root, "gui_backend/static")),
        [os.path.join(root, f) for f in files],
    )
    for root, _, files in os.walk("gui_backend/static")
    if files
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["static/*", "static/assets/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ]
    + static_files,
    install_requires=["setuptools", "fastapi", "uvicorn", "pyyaml"],
    zip_safe=False,
    maintainer="NODE Engineering Club",
    maintainer_email="node@example.org",
    description="FastAPI + WebSocket backend serving the Asket mission GUI.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gui_backend_node = gui_backend.gui_backend_node:main",
        ],
    },
)
