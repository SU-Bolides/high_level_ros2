from setuptools import find_packages, setup

package_name = 'navigation_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='voiture',
    maintainer_email='voiture@bolide.com',
    description='Master node to select between navigation algorithms and publish actuator commands',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_master = navigation_master.navigation_master:main',
        ],
    },
)