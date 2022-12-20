"""
Setup for accel_cases
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['accel_cases'],
        package_dir={'': 'src'}
    )

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'accel_cases'
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            (os.path.join('share', package_name), ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*.launch.py')),
            (os.path.join('share', package_name), ['config/settings.yaml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='The accel_cases package',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                '{}_node = {}.{}_node:main'.format(package_name, package_name, package_name)
            ],
        },
        package_dir={'': 'src'},
    )
