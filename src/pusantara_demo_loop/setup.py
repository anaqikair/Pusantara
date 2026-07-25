from setuptools import setup

package_name = "pusantara_demo_loop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orin_nano",
    maintainer_email="orin_nano@example.com",
    description="PUSANTARA full demo loop GUI using ROS2, Nav2 and heritage apps",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_loop = pusantara_demo_loop.demo_loop:main",
        ],
    },
)
