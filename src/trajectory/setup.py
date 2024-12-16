import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'trajectory'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools',
                      'rclpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='darius.kammawie@hotmail.com',
    description='tapping motion',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leftTapper = trajectory.left_tapper:main',
            'rightTapper = trajectory.right_tapper:main',
            'move = trajectory.move:main',
            'relax = trajectory.relax:main',
            'sensor_ft = trajectory.sensor_ft:main',
            'impedance_gain_adjuster = trajectory.impedance_control:main',
            'torsoLifter = trajectory.torso_lifter:main',
        ],
    },
)
