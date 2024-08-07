from setuptools import find_packages, setup

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics'  # 추가된 부분
    ],
    zip_safe=True,
    maintainer='kjy',
    maintainer_email='kjy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_package.camera_node:main',
            'test_node = camera_package.test_node:main'
        ],
    },
)
