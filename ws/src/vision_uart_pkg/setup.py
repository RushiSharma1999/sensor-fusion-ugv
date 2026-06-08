from setuptools import find_packages, setup

package_name = 'vision_uart_pkg'

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
    maintainer='cuirom',
    maintainer_email='cuirom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_uart_receiver_node = vision_uart_pkg.vision_uart_receiver_node:main',
            'camera_follow_node = vision_uart_pkg.camera_follow_node:main',
            'safety_mux_node = vision_uart_pkg.safety_mux_node:main',
        ],
    },
)
