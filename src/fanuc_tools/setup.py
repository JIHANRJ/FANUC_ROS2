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
            'monitor_state = fanuc_tools.monitor_state:main',
            'moveit_joint = fanuc_tools.moveit_joint:main',
            'moveit_cartesian = fanuc_tools.moveit_cartesian:main',
            'moveit_linear = fanuc_tools.moveit_linear:main',
            'moveit_go = fanuc_tools.moveit_go:main',
        ],
    },
)
