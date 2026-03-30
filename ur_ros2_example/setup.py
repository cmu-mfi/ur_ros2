from setuptools import find_packages, setup

package_name = 'ur_ros2_example'

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
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'servo_example = ur_ros2_example.servo_example:main',
            'move_example = ur_ros2_example.move_example:main',
            'io_example = ur_ros2_example.io_example:main',
            'tf_example = ur_ros2_example.tf_example:main',
        ],
    },
)
