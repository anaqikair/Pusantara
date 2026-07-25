from setuptools import setup
import os
from glob import glob

package_name = "vectra_odometry"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),

        (
            "share/" + package_name,
            ["package.xml"],
        ),

        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),

        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],

    install_requires=["setuptools"],

    zip_safe=True,

    maintainer="PUSANTARA",

    maintainer_email="you@example.com",

    description="Wheel encoder + IMU odometry",

    license="Apache-2.0",

    tests_require=["pytest"],

    entry_points={
        "console_scripts": [
            "odometry_node = vectra_odometry.odometry_node:main",
        ],
    },
)
