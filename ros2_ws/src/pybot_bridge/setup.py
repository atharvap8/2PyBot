import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pybot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "urdf"),
         glob(os.path.join("urdf", "*.urdf"))),
        (os.path.join("share", package_name, "config"),
         glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Atharva",
    maintainer_email="atharva@example.com",
    description="ROS 2 bridge for the 2PyBot self-balancing robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge = pybot_bridge.serial_bridge:main",
        ],
    },
)
