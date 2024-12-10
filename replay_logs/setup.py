from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'replay_logs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
      # Include resource idnex
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Include package.xml
        ('share/' + package_name, ['package.xml']),

        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Include config files
        (os.path.join('share', package_name, 'config'), ['config/ros_gazebo_topics.yaml']),

    ],
    install_requires=['setuptools', 'pandas'],
    zip_safe=True,
    maintainer='ruben',
    maintainer_email='paulo.ras98@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'replay = replay_logs.replay_logs:main'
        ],
    },
)
