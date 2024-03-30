import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ntrip_client'
setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],    #find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'], ),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'ntrip_client/ntrip_client_lib'), glob(
            os.path.join('ntrip_client/ntrip_client_lib', '__init__.py'))),
        (os.path.join('share', package_name, 'ntrip_client/ntrip_client_lib'), glob(
            os.path.join('ntrip_client/ntrip_client_lib', 'nmea_parser.py'))),
        (os.path.join('share', package_name, 'ntrip_client/ntrip_client_lib'), glob(
            os.path.join('ntrip_client/ntrip_client_lib', 'ntrip_client.py'))),
        (os.path.join('share', package_name, 'ntrip_client/ntrip_client_lib'), glob(
            os.path.join('ntrip_client/ntrip_client_lib', 'rtcm_parser.py'))),
    ],
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ties Junge',
    maintainer_email='t.junge@silolytics.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ntrip_node = ntrip_client.ntrip_node:main'
        ],
    },
)
