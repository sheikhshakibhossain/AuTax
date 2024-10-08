from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autax'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='sheikhshakibpro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_control = autax.base_control:main',
            'ai_agent = autax.ai_agent:main',
            'local_planner = autax.local_planner:main',
            'global_planner = autax.global_planner:main',
            'laser_scan = autax.lidar:main',
            'localization = autax.localization:main',
        ],
    },
)