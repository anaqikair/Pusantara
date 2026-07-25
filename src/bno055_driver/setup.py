from setuptools import setup

package_name = 'bno055_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin_nano',
    maintainer_email='you@example.com',
    description='BNO055 ROS2 Driver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = bno055_driver.imu_node:main',
        ],
    },
)
