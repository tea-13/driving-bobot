from setuptools import find_packages, setup

package_name = 'py_cmd_vel'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikita Zemlyanoi',
    maintainer_email='nikitazenlyanoi@gmail.com',
    description='The node that creates the cmd_vel topic when published to which type Twist messages are sent to the serial Arduino port.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'cmd_vel = py_cmd_vel.cmd_vel:main',
        ],
    },
)
