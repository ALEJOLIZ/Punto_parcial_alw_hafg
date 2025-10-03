from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trayectoria_parcial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'model'), glob("model/*.*")),
        (os.path.join("share", package_name,'dxf'), glob("dxf/*.*")),
        (os.path.join('share', package_name, 'mesh'), glob('mesh/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejo',
    maintainer_email='alejo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dk_node = trayectoria_parcial.dk_node:main',
            'ik_node = trayectoria_parcial.ik_node:main',
            'scara_state_publisher = trayectoria_parcial.scara_state_publisher:main',
            'traj_guide_node = trayectoria_parcial.traj_guide_node:main',
            'traj_plan_node = trayectoria_parcial.traj_plan_node:main',
            'goal_translator = trayectoria_parcial.goal_translator:main',
            'dxf_exporter_node = trayectoria_parcial.dxf_exporter_node:main',
        ],
    },
)
