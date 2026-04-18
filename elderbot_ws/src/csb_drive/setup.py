import os
from glob import glob

from setuptools import find_packages, setup


package_name = "csb_drive"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="ztl",
    maintainer_email="ztl@todo.todo",
    description="Serial driver for the DYP-E08 ultrasonic radar.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_pyserial = csb_drive.drive_pyserial:main",
        ],
    },
)
