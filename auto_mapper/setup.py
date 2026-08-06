from setuptools import setup

package_name = 'auto_mapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masumi',
    maintainer_email='mylifestyle.mh9482@gmail.com',
    description='Fully autonomous indoor mapping node (frontier + LiDAR-ray + wander)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_explorer = auto_mapper.auto_explorer:main',
        ],
    },
)
