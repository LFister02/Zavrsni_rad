from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'a_star_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*_launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucijan Fišter',
    maintainer_email='lucijan.fister02@gmail.com',
    description='Testiranje i razvoj A* algoritma za korištenje unutar simulacije i na stvarnoj letjelici',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'a_star = a_star_test.a_star:main', 
        	'gazebo_follower = a_star_test.sim_follower:main',
            'gazebo_a_star = a_star_test.octomap_a_star:main',
            'save_pointcloud = a_star_test.save_pointcloud:main',
            'tello_map_real = a_star_test.tello_map_real:main',
            'follower_dyn_obs = a_star_test.follower_dyn_obs:main',
            'scan_to_range = a_star_test.scan_to_range:main',
            'crta_follower = a_star_test.crta_follower:main',
            'gazebo_dyn_obs = a_star_test.gazebo_dyn_obs:main',
        ],
    },
)
