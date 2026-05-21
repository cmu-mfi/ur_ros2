from setuptools import find_packages, setup

package_name = 'ur_examples'

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
            'joint_goal_example = ur_examples.joint_goal_example:main',
            'pose_goal_example = ur_examples.pose_goal_example:main',
            'servo_force = ur_examples.servo_force:main',
            'io_example = ur_examples.io_example:main',
        ],
    },
)
