from setuptools import setup

package_name = 'radar_graph_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koide',
    maintainer_email='koide@aisl.cs.tut.ac.jp',
    description='4DRadarSLAM ROS 2 port',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'map2odom_publisher = radar_graph_slam.map2odom_publisher:main',
            'bag_player = radar_graph_slam.bag_player:main',
            'ford2bag = radar_graph_slam.ford2bag:main',
        ],
    },
)
