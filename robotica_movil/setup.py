from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotica_movil'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='a01798048@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoders_velocity = robotica_movil.encoders_velocity:main',
            'odometry = robotica_movil.odometry:main',
            'waypoints_generator = robotica_movil.waypoints_generator:main',
            'trajectory_generator = robotica_movil.trajectory_generator:main',
            'robot_controller = robotica_movil.robot_controller:main',
            'turn_while_go = robotica_movil.turn_while_go:main',
            'turn_then_go = robotica_movil.turn_then_go:main',
            'pure_pursuit = robotica_movil.pure_pursuit:main',
            'lyapunov = robotica_movil.lyapunov:main',
            'turtle_pose = robotica_movil.turtle_pose:main',
        ],
    },
)
