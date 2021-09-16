import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rosbridge_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir = {'':'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Administrator',
    maintainer_email='396431649@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbridge_tcp = rosbridge_server.scripts.rosbridge_tcp:main',
            'rosbridge_udp = rosbridge_server.scripts.rosbridge_udp:main',
            'rosbridge_websocket = rosbridge_server.scripts.rosbridge_websocket:main'
        ],
    },
)
