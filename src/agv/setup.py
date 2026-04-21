import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agv'


def pkg_files(subdir):
    return [f for f in glob(os.path.join(subdir, '**', '*'), recursive=True)
            if os.path.isfile(f)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), pkg_files('urdf')),
        (os.path.join('share', package_name, 'meshes'), pkg_files('meshes')),
        (os.path.join('share', package_name, 'textures'), pkg_files('textures')),
        (os.path.join('share', package_name, 'config'), pkg_files('config')),
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
            'spawn_agv = agv.spawn_agv:main',
        ],
    },
)
