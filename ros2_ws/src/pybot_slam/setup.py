import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pybot_slam"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Atharva",
    maintainer_email="atharva@example.com",
    description="PLACEHOLDER: lidar SLAM for 2PyBot",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
