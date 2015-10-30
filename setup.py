#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tactile_marker_publisher','tactile_merger'],
    package_dir={'': 'src'},
    scripts=['scripts/tactile_marker_publisher','scripts/tactile_merger'],
    requires=['genmsg', 'genpy', 'roslib', 'rospkg', 'visualization_msgs', 'tf', 'numpy', 'webcolors']
)

setup(**d)
