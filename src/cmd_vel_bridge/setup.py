from setuptools import find_packages, setup

package_name = "cmd_vel_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin_nano",
    maintainer_email="orin_nano@example.com",
    description="Converts /cmd_vel Twist messages into /robot_command string motor commands.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_to_robot_command = cmd_vel_bridge.cmd_vel_to_robot_command:main",
        ],
    },
)
