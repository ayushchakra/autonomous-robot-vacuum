from setuptools import setup

package_name = 'robot_vacuum'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='achakraborty@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laptop_sender = robot_vacuum.laptop_interface:main',
            'rpi_receiver = robot_vacuum.rpi_interface:main',
            'obstacle_avoidance = robot_vacuum.obstacle_avoidance:main'
        ],
    },
)
