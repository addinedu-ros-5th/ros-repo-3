from setuptools import setup

package_name = 'rfid_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yokim',
    maintainer_email='yokim@todo.todo',
    description='RFID Publisher Node for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rfid_publisher_node = rfid_reader.rfid_publisher_node:main',
            'rfid_subscriber_node = rfid_reader.rfid_subscriber_node:main'
        ],
    },
)
