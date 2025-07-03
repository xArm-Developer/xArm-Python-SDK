xarm-python-sdk
===============

.. image:: https://badge.fury.io/py/xarm-python-sdk.svg
    :target: https://pypi.org/project/xarm-python-sdk/

.. image:: https://static.pepy.tech/badge/xarm-python-sdk
    :target: https://pepy.tech/projects/xarm-python-sdk

.. image:: https://img.shields.io/github/license/xArm-Developer/xArm-Python-SDK.svg
    :target: https://github.com/xArm-Developer/xArm-Python-SDK/blob/main/LICENSE

.. image:: https://img.shields.io/pypi/pyversions/xarm-python-sdk.svg
    :target: https://pypi.org/project/xarm-python-sdk/

Official Python SDK for UFACTORY robots.

Supported Products
------------------

- UFACTORY xArm 5/6/7
- UFACTORY 850
- UFACTORY Lite 6

Compatibility
-------------

- Python 3.5 - 3.13

Installation
------------

Install from PyPI:

.. code-block:: bash

    pip install xarm-python-sdk

Quick Start
-----------

.. code-block:: python

    from xarm.wrapper import XArmAPI

    # Connect to the robot
    arm = XArmAPI('192.168.1.100')  # Replace with your robot's IP address

    # Enable motion
    arm.motion_enable(enable=True)

    # Set robot to ready state
    arm.set_mode(0)    # Position control mode
    arm.set_state(0)   # Set to ready state

    # Move to a target position
    arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, speed=100)

    # Disconnect
    arm.disconnect()

Documentation
-------------

Full documentation and examples are available at:

https://github.com/xArm-Developer/xArm-Python-SDK

Website: https://www.ufactory.cc/

Release Note: https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/README.md#update-summary

Bug Reports: support@ufactory.cc

License
-------

- License: BSD 3-Clause License. See `LICENSE <https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/LICENSE>`_ for details.
