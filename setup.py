#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
            packages=['moticon_insoles', 'insoles_common','buffer'],
            package_dir={'':'src'}
        )
setup(**setup_args)
