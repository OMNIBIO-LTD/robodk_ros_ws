from setuptools import find_packages, setup

package_name = 'goal_pose_cmd'

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
    maintainer='qasob',
    maintainer_email='qasobovgholib@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_pose_cmd_node      = goal_pose_cmd.goal_pose_cmd_node:main',
            'read_tcp_pose_node      = goal_pose_cmd.read_tcp_pose:main',
            'joint_state_streamer    = goal_pose_cmd.joint_state_streamer:main',
            'gripper_button          = goal_pose_cmd.gripper_button:main',
        ],
    },
)
