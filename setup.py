#!/usr/bin/env python3
# BSD License (c) 2017, UFactory, Inc.
# Author: Vinman <vinman.wen@ufactory.cc>

import os

# Prefer setuptools but fall back to distutils if needed
try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup
    
# Load version from xarm/version.py
main_ns = {}
with open(os.path.join(os.getcwd(), 'xarm/version.py')) as f:
    exec(f.read(), main_ns)
version = main_ns['__version__']

# Read README.rst and requirements.txt
with open('README.rst', encoding='utf-8') as f:
    long_description = f.read()
with open(os.path.join(os.getcwd(), 'requirements.txt'), encoding='utf-8') as f:
    requirements = f.read().splitlines()

setup(
    name="xarm-python-sdk",
    version=version,
    packages=['xarm'],
    install_requires=requirements,
    author="Vinman <vinman.wen@ufactory.cc>",
    description="Python SDK for xArm Robotics",
    long_description=long_description,
    python_requires=">=3.0",
)
