from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'homework_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vladimir',
    maintainer_email='vladimir@todo.todo',
    description='Homework 1 ROS 2 package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'senzor_temperatura = homework_1.senzor_temperatura:main',
            'senzor_vlaznost = homework_1.senzor_vlaznost:main',
            'senzor_osvetljenje = homework_1.senzor_osvetljenje:main',
            'vizuelizacija_sistema = homework_1.vizuelizacija_sistema:main',
            'brava = homework_1.brava:main',
            'alarm = homework_1.alarm:main',
        ],
    },
)
