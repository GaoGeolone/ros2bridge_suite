from setuptools import setup, find_packages

package_name = 'rosbridge_library'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir = {'':'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Administrator',
    maintainer_email='396431649@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    extras_require = {
        'rosbridge_protocol': [
            'pymongo>=3.12.0',
            'Pillow>=8.3.2',
            'twisted>=21.7.0'
            ]
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
