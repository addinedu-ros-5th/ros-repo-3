from setuptools import find_packages, setup

package_name = 'odbot_astar_movement'

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
    maintainer='hb',
    maintainer_email='rowl126@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'astar_movement = odbot_astar_movement.astar_movement:main',
        'command_publisher = odbot_astar_movement.command_publisher:main',
        ],
    },
)
