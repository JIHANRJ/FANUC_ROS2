import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fanuc_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Motion launch + config
        (os.path.join('share', package_name, 'motion'),
            glob('launch/motion/*.launch.py')),
        (os.path.join('share', package_name, 'motion'),
            glob('config/motion/*.yaml')),
        # TCP launch + config
        (os.path.join('share', package_name, 'tcp'),
            glob('launch/tcp/*.launch.py')),
        (os.path.join('share', package_name, 'tcp'),
            glob('config/tcp/*.yaml')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jihanrj',
    maintainer_email='jihanrj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'monitor_state = fanuc_tools.legacy.monitor_state:main',
            'moveit_joint = fanuc_tools.legacy.moveit_joint:main',
            'moveit_cartesian = fanuc_tools.legacy.moveit_cartesian:main',
            'moveit_linear = fanuc_tools.legacy.moveit_linear:main',
            'moveit_go = fanuc_tools.legacy.moveit_go:main',
            'test_program = fanuc_tools.legacy.test_program:main',
            'pick_place = fanuc_tools.legacy.pick_place:main',
            'move_joint = fanuc_tools.motion.move_joint:main',
            'modular_joint_demo = fanuc_tools.motion.modular_joint_demo:main',
            'speed_scaling = fanuc_tools.motion.speed_scaling:main',
            'collaborative_speed = fanuc_tools.motion.collaborative_speed:main',
            'move_cartesian = fanuc_tools.motion.move_cartesian:main',
            'move_linear = fanuc_tools.motion.move_linear:main',
            'jog_ps4 = fanuc_tools.motion.jog_ps4:main',
        ],
    },
)
