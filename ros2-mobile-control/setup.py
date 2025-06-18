from setuptools import setup

package_name = 'ros2_mobile_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'flask-socketio'],
    zip_safe=True,
    maintainer='Nagendra Sriram',
    maintainer_email='nagendra3feb@gmail.com',
    description='Mobile teleoperation interface for ROS2 using gyroscope and buttons.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_mobile_control = ros2_mobile_control.main:main'
        ],
    },
)
