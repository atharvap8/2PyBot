import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pybot_teleop"

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
        (os.path.join("share", package_name, "config"),
         glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Atharva",
    maintainer_email="atharva@example.com",
    description="EvoFox One S gamepad teleop for 2PyBot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_teleop = pybot_teleop.joy_teleop:main",
        ],
    },
)
