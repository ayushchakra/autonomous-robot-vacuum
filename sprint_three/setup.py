from setuptools import setup

package_name = 'sprint_three'

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
            'laptop_sender = sprint_three.laptop_sender:main',
            'rpi_receiver = sprint_three.rpi_receiver:main',
        ],
    },
)
