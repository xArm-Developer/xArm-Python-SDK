#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
import functools
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.tools.xml_tool import XmlTool

# blockly app path
source_path = 'C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm7\\app\\myapp\\233\\app.xml'
# the path is the python code to save
target_path = './py_blockly.py'
xml = XmlTool(source_path)
xml.to_python(target_path)
