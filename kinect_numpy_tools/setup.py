#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['kinect_numpy_tools'],
    package_dir={'': 'src'},
    requires=['genpy', 'numpy', 'rosgraph', 'roslib', 'rospkg']
)

setup(**d)
