from setuptools import setup

package_name = 'object_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m-horiuchi',
    maintainer_email='mylifestyle.mh9482@gmail.com',
    description='Object patrol robot: patrol a known map and locate objects with YOLO + LiDAR fusion',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'yolo_detector = object_patrol.yolo_detector:main',
        'set_initial_pose = object_patrol.set_initial_pose:main',
    ]},
)
