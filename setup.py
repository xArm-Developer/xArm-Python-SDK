#!/usr/bin/env python3
import os
import sys
install_setuptools = False
try:
    from setuptools import setup, find_packages
except ImportError:
    os.system('{} -m pip install setuptools'.format(sys.executable))
    # os.system('{} -m pip install setuptools==79.0.0'.format(sys.executable))
    install_setuptools = True
    try:
        from setuptools import setup, find_packages
    except ImportError as e:
        print('setuptools not found, {}'.format(e))
        exit(1)

main_ns = {}
with open(os.path.join(os.getcwd(), 'xarm/version.py')) as ver_file:
    exec(ver_file.read(), main_ns)
    version = main_ns['__version__']

long_description = open('README.rst', encoding='utf-8').read()

requirements_path = os.path.join(os.getcwd(), 'requirements.txt')
if os.path.exists(requirements_path):
    with open(os.path.join(os.getcwd(), 'requirements.txt')) as f:
        requirements = f.read().splitlines()
else:
    requirements = []

try:
    setup(
        name='xarm-python-sdk',
        version=version,
        author='Vinman',
        author_email='vinman.wen@ufactory.cc',
        description='Python SDK for UFACTORY robotic arm 850, xArm 5/6/7, and Lite6.',
        long_description=long_description,
        url='https://github.com/xArm-Developer/xArm-Python-SDK',
        packages=find_packages(),
        install_requires=requirements,
        # license='BSD',
        zip_safe=False,
        classifiers=[
            "Intended Audience :: Developers",
            "License :: OSI Approved :: BSD License",
            "Operating System :: OS Independent",
            "Programming Language :: Python",
            "Programming Language :: Python :: 3",
            "Programming Language :: Python :: 3.5",
            "Programming Language :: Python :: 3.6",
            "Programming Language :: Python :: 3.7",
            "Programming Language :: Python :: 3.8",
            "Programming Language :: Python :: 3.9",
            "Programming Language :: Python :: 3.10",
            "Programming Language :: Python :: 3.11",
            "Programming Language :: Python :: 3.12",
            "Programming Language :: Python :: 3.13",
            "Topic :: Software Development",
        ],
        python_requires='>=3.5',
    )
except Exception as e:
    raise e
finally:
    if install_setuptools:
        os.system('{} -m pip uninstall setuptools -y'.format(sys.executable))
