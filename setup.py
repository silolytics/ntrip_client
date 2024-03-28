import os
import glob
from setuptools import setup, find_packages

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={'': 'src'},
    #data_files=[
    #    ('share/ament_index/resource_index/packages',
    #        ['resource/' + package_name]),
    #    ('share/' + package_name, ['package.xml']),
    #    #(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
#
    #],
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ties Junge',
    maintainer_email='info@silolytics.de',
    maintainer='Ties Junge',
    maintainer_email='info@silolytics.de',
    description='NTRIP client that will publish RTCM corrections to a ROS topic, and optionally subscribe to NMEA messages to send to an NTRIP server',
    license='MIT License',
    tests_require=['pytest'],
    scripts=[
        'scripts/ntrip_ros_lifecycle.py'
    ]
)
