from setuptools import find_packages, setup

package_name = 'lite6_velocity'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/lw3_circles.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cazevedo',
    maintainer_email='cazevedo@ipn.pt',
    description='LW3: velocity control of the UFACTORY Lite 6 along a circle (Robotics 02000537).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vel_node = lite6_velocity.velocity_control:main',
        ],
    },
)
