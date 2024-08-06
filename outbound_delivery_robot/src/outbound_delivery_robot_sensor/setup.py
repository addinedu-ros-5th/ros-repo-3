from setuptools import find_packages, setup

package_name = 'outbound_delivery_robot_sensor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'RPi.GPIO', 'mfrc522'],
    zip_safe=True,
    maintainer='yokim',
    maintainer_email='yokim@todo.todo',
    description='RFID Publisher Node for ROS 2 and Ultrasonic Publisher Node for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rfid_publisher_node = rfid_reader.tag_reader:main',
            'ultrasonic_sensor_node = ultrasonic_sensor.ultrasonic_sensor_node:main',
            'ultrasonic_sensor = ultrasonic_sensor.ultrasonic_sensor:main',
        ],
    },
)
