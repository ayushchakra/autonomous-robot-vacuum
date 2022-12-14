from setuptools import setup

package_name = 'rc_car'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "receive_message = rc_car.connection_test.receive_message:main",
            "send_message = rc_car.connection_test.send_message:main",
            "send_lidar = rc_car.send_lidar:main",
            "receive_lidar = rc_car.receive_lidar:main",
            "send_ip = rc_car.rpi_startup.send_ip_node:main",
            "receive_ip = rc_car.rpi_startup.receive_ip_node:main"
        ],
    },
)
