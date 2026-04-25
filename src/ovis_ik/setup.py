from setuptools import find_packages, setup

package_name = 'ovis_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'numpy', 'roboticstoolbox-python'],
    zip_safe=True,
    maintainer='capra',
    maintainer_email='capra@ens.etsmtl.ca',
    description='Inverse differential kinematics nodes for the Ovis arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_node = ovis_ik.velocity_node:main',
            'ovis_viz_node = ovis_ik.ovis_viz_node:main',
            'constant_vel_publisher = ovis_ik.constant_vel_publisher:main',
            'fk_simulation_node = ovis_ik.fk_simulation_node:main',
        ],
    },
)
