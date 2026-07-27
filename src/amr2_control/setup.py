from setuptools import find_packages, setup
 
package_name = 'amr2_control'
 
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonyeejun',
    maintainer_email='dlwns2636@gamil.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_manager = amr2_control.amr2_state_manager:main',
            'camera_pub = amr2_control.amr2_camera_pub:main',
            'navigator = amr2_control.amr2_navigator:main',
            'controller = amr2_control.amr2_controller:main',
        ],
    },
)
