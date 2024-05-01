## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# from setuptools import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# # fetch values from package.xml
# setup_args = generate_distutils_setup(
#     packages=['kinova_demo'],
#     package_dir={'': 'ros2_nodes'},
#     requires=['std_msgs', 'rclcpp', 'rclpy', 'kinova_msgs', 'kinova_driver', 'geometry_msgs']
# )

# setup(**setup_args)

from setuptools import setup, find_packages, setup, os

package_name = 'kinova_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'ros2_nodes'},
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Your package description',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joints_action_client = kinova_demo.joints_action_client:main',  # Ajout de cette ligne
        ],
    },
)
