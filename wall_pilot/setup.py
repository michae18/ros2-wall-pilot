from setuptools import setup

package_name = 'wall_pilot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='min',
    maintainer_email='example@example.com',
    description='Wall detection and pilot nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wall_detector = wall_pilot.wall_detector:main',
            'wall_pilot    = wall_pilot.wall_pilot_node:main',
            'wall_detector_v2 = wall_pilot.wall_detector_v2:main',
        ],
    },
)