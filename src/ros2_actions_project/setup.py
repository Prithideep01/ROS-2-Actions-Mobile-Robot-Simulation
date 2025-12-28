from setuptools import find_packages, setup

package_name = 'ros2_actions_project'

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
    maintainer='faps',
    maintainer_email='fadhsing@faps.uni-erlangen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_server = ros2_actions_project.robot_server:main",
            "robot_client = ros2_actions_project.robot_client:main"
        ],
    },
)
