from setuptools import setup
import os
from glob import glob

package_name = 'aeron'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/aeron_robot'), ['models/aeron_robot/model.sdf']),
        (os.path.join('share', package_name, 'scripts'), ['scripts/joystick_controller.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gautam',
    maintainer_email='gautam@example.com',
    description='AERON TARS simulation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joystick_controller = aeron.joystick_controller:main',
        ],
    },
)from setuptools import setup
import os
from glob import glob

package_name = 'aeron'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models/aeron_robot'), ['models/aeron_robot/model.sdf']),
        (os.path.join('share', package_name, 'scripts'), ['scripts/joystick_controller.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gautam',
    maintainer_email='gautam@example.com',
    description='AERON TARS simulation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joystick_controller = aeron.joystick_controller:main',
        ],
    },
)
