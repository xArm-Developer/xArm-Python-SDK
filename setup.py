#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>

import os
from distutils.util import convert_path
try:
    from setuptools import setup, find_packages
except ImportError:
    from distutils.core import setup

    def find_packages(base_path='.'):
        base_path = convert_path(base_path)
        found = []
        for root, dirs, files in os.walk(base_path, followlinks=True):
            dirs[:] = [d for d in dirs if d[0] != '.' and d not in ('ez_setup', '__pycache__')]
            relpath = os.path.relpath(root, base_path)
            parent = relpath.replace(os.sep, '.').lstrip('.')
            if relpath != '.' and parent not in found:
                # foo.bar package but no foo package, skip
                continue
            for dir in dirs:
                if os.path.isfile(os.path.join(root, dir, '__init__.py')):
                    package = '.'.join((parent, dir)) if parent else dir
                    found.append(package)
        return found

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
    author_email='vinman@ufactory.cc',
    install_requires=requirements,
    long_description=long_description,
    license='MIT',
    zip_safe=False
)
