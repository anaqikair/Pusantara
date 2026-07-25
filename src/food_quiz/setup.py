from setuptools import find_packages, setup

package_name = 'food_quiz'

setup(
name=package_name,
version='0.0.0',
packages=find_packages(exclude=['test']),
data_files=[
(
'share/ament_index/resource_index/packages',
['resource/' + package_name]
),
(
'share/' + package_name,
['package.xml']
),
(
'share/' + package_name + '/images',
[
'resource/cendol.jpg',
'resource/nasi_kerabu.jpg',
'resource/satay.jpg',
'resource/tehtarik.jpg'
]
),
],
install_requires=[
'setuptools'
],
zip_safe=True,
maintainer='orin_nano',
maintainer_email='[orin_nano@todo.todo](mailto:orin_nano@todo.todo)',
description='Malaysian Food Heritage Quiz',
license='MIT',
tests_require=['pytest'],
entry_points={
'console_scripts': [
'food_quiz = food_quiz.foodquiz:main',
],
},
)

