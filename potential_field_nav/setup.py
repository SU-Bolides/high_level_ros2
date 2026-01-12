import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'potential_field_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # Install the executable wrapper into lib/<package> so ros2 Node can find it
        (os.path.join('lib', package_name), ['scripts/potential_field']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='voiture',
    maintainer_email='voiture@bolide.com',
    description='Potential Field navigator package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potential_field=potential_field_nav.potential_field_navigator:main',
        ],
    },
)