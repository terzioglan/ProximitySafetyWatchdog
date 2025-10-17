from setuptools import find_packages, setup

package_name = 'safety_watchdog'

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
    description='Contains a robot controller safety gate that sets the final robot velocity based on proximity and emergency button status.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'safety_gate_robot = safety_watchdog.safety_gate_robot:main',
            'safety_gate_turtle = safety_watchdog.safety_gate_turtle:main'
        ],
    },
)
