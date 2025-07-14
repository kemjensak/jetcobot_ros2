import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jetcobot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jinseok Kim',
    maintainer_email='jinseok.kim970@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_control = jetcobot_bringup.joint_control:main',
            'camera_info_publisher = jetcobot_bringup.camera_info_publisher:main',
        ],
    },
)
