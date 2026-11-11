from setuptools import find_packages, setup

package_name = 'fw_lqi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SirPicklee',
    maintainer_email='258267710+SirPicklee@users.noreply.github.com',
    description='LQI offboard flight controller for the PX4 fixed-wing SITL.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
        ],
    },
)
