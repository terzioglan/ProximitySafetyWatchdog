from setuptools import find_packages, setup

package_name = 'safety_monitor'

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
    maintainer='terzi',
    maintainer_email='terziogluyunus@gmail.com',
    description='Contains a node to simulate the big red emergency stop button to bring a robot to a halt.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'red_button = safety_monitor.red_button:main',
            'proximity_sensor = safety_monitor.proximity_sensor:main',
        ],
    },
    # py_modules=[package_name+'/config']
)
