from setuptools import find_packages, setup

package_name = 'lite6_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/lw1_poses.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cazevedo',
    maintainer_email='cazevedo@ipn.pt',
    description='LW1: forward kinematics of the UFACTORY Lite 6 (Robotics 02000537).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_check = lite6_kinematics.fk_check:main',
        ],
    },
)
