from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hello_world'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etfrobot',
    maintainer_email='etfrobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = hello_world.talker:main', 
            'listener = hello_world.listener:main',
            'add_server = hello_world.add_server:main',
            'add_client = hello_world.add_client:main',
        ],
    },
)
