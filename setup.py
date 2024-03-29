import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ntrip_client'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'], ),
        ('share/' + package_name + '/config',
            ['config/ntrip_client.yaml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
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
