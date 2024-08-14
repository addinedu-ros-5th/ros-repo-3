from setuptools import find_packages, setup

package_name = 'outbound_delivery_robot_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='kjy',
    maintainer_email='jykang560@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pub_node = outbound_delivery_robot_camera.camera_pub_node:main',
            'camera_sub_node = outbound_delivery_robot_camera.camera_sub_node:main',
            'test_node = outbound_delivery_robot_camera.test_node:main'
        ],
    },
)
