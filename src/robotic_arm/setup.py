import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch') + glob('launch/*.launch.py'),
        ),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.csv')),
        (
            os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.STL') + glob('meshes/*.stl'),
        ),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'textures', 'Rack'), glob('textures/Rack/*.dae') + glob('textures/Rack/*.jpg') + glob('textures/Rack/*.png')),
        (os.path.join('share', package_name, 'textures', 'box'), glob('textures/box/*.dae') + glob('textures/box/*.jpg') + glob('textures/box/*.png')),
        (os.path.join('share', package_name, 'Warehouse'), glob('Warehouse/*.dae') + glob('Warehouse/*.jpg') + glob('Warehouse/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raghav',
    maintainer_email='raghav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_pose_detector = robotic_arm.aruco_pose_detector:main',
        ],
    },
)
