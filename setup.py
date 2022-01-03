from setuptools import setup

package_name = 'joycon_controller'

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
    maintainer='Azicode',
    maintainer_email='azi.3018r@gmail.com',
    description='Control Turtlesim by Joy Controller',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joycon_controller = '+ package_name + '.joycon_controller:ros_main',
        ],
    },
)
