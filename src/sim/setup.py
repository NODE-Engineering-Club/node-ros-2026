import os
from glob import glob
from setuptools import setup

package_name = 'sim'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # config / rviz files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NODE',
    maintainer_email='node@iaac.net',
    description='2D Python simulator for Asket ASV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simulator = sim.simulator:main',
        ],
    },
)
