from setuptools import setup

package_name = 'px4_to_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [ 'launch/launch_fly.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='fabiwenmeier@gmail.com',
    description='PX4 to ROS2 Python interface',
    license='MIT',
    entry_points={
        'console_scripts': [
            'px4=px4_to_ros2.px4_convert:main',
        ],
    },
)