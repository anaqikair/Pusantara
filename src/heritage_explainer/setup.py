from setuptools import setup

package_name = 'heritage_explainer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin_nano',
    maintainer_email='robot@pusantara.local',
    description='PUSANTARA Heritage AprilTag Explainer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heritage_explainer = heritage_explainer.heritage_explainer:main',
        ],
    },
)
