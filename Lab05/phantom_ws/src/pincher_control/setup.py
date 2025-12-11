from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'pincher_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_servo = pincher_control.control_servo:main',
            'control_servo2 = pincher_control.control_servo2:main',
            'Lab5_P1 = pincher_control.Lab5_P1 :main',
            'terminal_control = pincher_control.terminal_control :main',
            'terminal_suscriber = pincher_control.terminal_suscriber :main',
            'toolbox = pincher_control.toolbox :main',
            'HMI_RVIZ = pincher_control.HMI_RVIZ :main',


        ],
    },
)