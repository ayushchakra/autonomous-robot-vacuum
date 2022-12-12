from setuptools import setup

package_name = 'sprint_two'

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
    maintainer='achakraborty',
    maintainer_email='achakraborty@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_teleop = sprint_two_rc.send_teleop:main',
            'simple_send_teleop = sprint_two.simple_send_teleop:main',
            'simple_receive_teleop = sprint_two.simple_receive_teleop:main',
            'obstacle_avoidance = sprint_two.obstacle_avoidance:main',
        ],
    },
)
