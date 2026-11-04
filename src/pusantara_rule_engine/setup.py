from setuptools import setup
from glob import glob
import os

package_name = "pusantara_rule_engine"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Pusantara Team",
    maintainer_email="team@example.com",
    description="YAML-driven generic ROS 2 topic rule engine",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "engine = pusantara_rule_engine.rule_engine:main"
    ]},
)
