from setuptools import find_packages, setup

package_name = 'outbound_delivery_robot_movement'

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
        'astar_movement = outbound_delivery_robot_movement.astar_movement:main',
        'command_publisher = outbound_delivery_robot_movement.command_publisher:main',
        'command_planned = outbound_delivery_robot_movement.command_planned:main',
        'command_path = outbound_delivery_robot_movement.command_path:main',
        ],
    },
)
