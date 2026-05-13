from setuptools import setup

package_name = 'radar_graph_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='you@example.com',
    description='4DRadarSLAM ROS 2 port',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'map2odom_publisher = radar_graph_slam.map2odom_publisher:main',
        ],
    },
)