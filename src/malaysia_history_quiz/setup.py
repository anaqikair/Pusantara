from setuptools import setup
import os
from glob import glob

package_name = 'malaysia_history_quiz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name,
            ['package.xml']),

        (
            os.path.join('share', package_name, 'images'),
            glob('images/*')
        ),

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orin_nano',
    maintainer_email='robot@localhost',
    description='Malaysian History Quiz',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'history_quiz = malaysia_history_quiz.history_quiz:main',
        ],
    },
)
