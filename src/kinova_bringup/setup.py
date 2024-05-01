from setuptools import setup
import os
from glob import glob

package_name = 'kinova_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},  # Chemin vers les fichiers de votre package
    install_requires=['setuptools'],
    zip_safe=True,
    author='Votre Nom',
    author_email='votre.email@example.com',
    maintainer='Votre Nom',
    maintainer_email='votre.email@example.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Votre description de package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
)
