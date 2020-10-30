#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ta11_test', 'ta11_tiago'],
    package_dir={'': 'src'}
)

setup(**d)
