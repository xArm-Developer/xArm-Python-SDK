#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>


try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup
    def find_packages():
        return ['xarm', 'xarm.core', 'xarm.core.comm', 'xarm.core.utils', 'xarm.core.wrapper', 'xarm.wrapper']
import os
from distutils.util import convert_path

main_ns = {}
ver_path = convert_path('xarm/version.py')
with open(os.path.join(os.getcwd(), ver_path)) as ver_file:
    exec(ver_file.read(), main_ns)

version = main_ns['__version__']

# long_description = open('README.rst').read()
long_description = 'long description for xArm-Python-SDK'

with open(os.path.join(os.getcwd(), 'requirements.txt')) as f:
    requirements = f.read().splitlines()

setup(
    name='xArm-Python-SDK',
    version=version,
    author='Vinman',
    description='Python SDK for xArm',
    packages=find_packages(),
    author_email='developer@ufactory.cc',
    install_requires=requirements,
    long_description=long_description,
    license='MIT',
    zip_safe=False
)
