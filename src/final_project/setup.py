from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={
        package_name: ['templates/*.html'],
    },
    include_package_data=True,
    install_requires=[
        'setuptools',
    ],

    zip_safe=True,
    maintainer='rokey',
    maintainer_email='seungyeon_oh@hotmail.com',
    description='AMR1 Monitor and Navigation package for the final project',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_tracker=final_project.yolo_tracker_v4:main',
            'monitor=final_project.monitor1_v3_battery_alert:main',
            'navigation=final_project.navigation_v5_cmd_vel:main',
            'amr_alarm=final_project.amr_alarm_node:main',
        ],
    },
)
