from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'ros2_pyimclsts'
submodule_imc_base = 'ros2_pyimclsts/pyimc_generated'
submodule_imc_categories = 'ros2_pyimclsts/pyimc_generated/categories'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule_imc_base, submodule_imc_categories],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), ['config/ros_gazebo_topics.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruben',
    maintainer_email='paulo.ras98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros-python-client = ros2_pyimclsts.udp_connection:main',
            'gazebo-ros-dune   = ros2_pyimclsts.receive_state:main'
        ],
    },
)
