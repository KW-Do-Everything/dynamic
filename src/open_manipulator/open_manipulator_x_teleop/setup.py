from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_x_teleop'

setup(
    name=package_name,
    version='2.3.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Will Son',
    author_email='willson@robotis.com',
    maintainer='Will Son',
    maintainer_email='willson@robotis.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation node using keyboard for OpenMANIPULATOR-X.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = open_manipulator_x_teleop.script.teleop_keyboard:main',
            'manipulator_controller = open_manipulator_x_teleop.script.test_node:main'
        ],
    },
)
