from setuptools import find_packages, setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yassa Emad',
    maintainer_email='your_email@example.com',
    description='goToPose, reset, parameter and square servers for turtlesim.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_pose_server = turtle_control.go_to_pose_server:main',
            'reset_service = turtle_control.reset_service:main',
            'draw_square_server = turtle_control.draw_square_server:main',
        ],
    },
)
