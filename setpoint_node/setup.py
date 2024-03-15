from glob import glob
from setuptools import find_packages, setup

package_name = 'reto'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/setpoint_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='rodrigollagunoc@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Setpoint = reto.Setpoint:main'
        ],
    },
)
