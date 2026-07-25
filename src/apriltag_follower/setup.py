from setuptools import setup

package_name = 'apriltag_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotik',
    maintainer_email='user@example.com',
    description='AprilTag follower for PUSANTARA',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_follower = apriltag_follower.apriltag_follower:main',
        ],
    },
)
