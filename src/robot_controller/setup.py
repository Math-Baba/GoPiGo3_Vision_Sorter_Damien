from setuptools import find_packages, setup

package_name = 'robot_controller'

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
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Robot controller for cube sorting',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'gopigo3_driver = robot_controller.gopigo3_driver:main',
            'cube_detector = robot_controller.cube_detector:main',
            'imu_node = robot_controller.imu_node:main',
            'dashboard_node = robot_controller.dashboard_node:main',
            'cube_strategy = robot_controller.cube_strategy:main',
        ],
    },
)
