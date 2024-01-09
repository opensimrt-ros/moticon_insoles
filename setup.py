#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
            packages=['moticon_insoles', 'insoles_common','buffer','better_rate','insole_data_from_file','insole_data_from_socket','insole_data_getter','insole_srv','side_startup_data','utils'],
            package_dir={'':'src'}
        )
setup(**setup_args)
