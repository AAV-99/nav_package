from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir carpeta config completa
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        #Incluir archivos launch
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        
        # Incluir mapas
        (os.path.join('share', package_name, 'maps'), 
         glob('maps/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aav',
    maintainer_email='aav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_poses = nav_package.nav_to_poses:main',
            'orchestrator_node = nav_package.orchestrator_node:main',
            'mqtt_bridge = nav_package.mqtt_bridge:main',
            'docking_node = nav_package.docking_node:main',
            'orchestrator = nav_package.orchestrator:main',
            'nav = nav_package.nav:main',

        ],
    },
)

