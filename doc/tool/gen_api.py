#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, UFactory, Inc.
# All rights reserved.
#
# Author: Duke Fong <duke@ufactory.cc>


import sys
import os
import pydoc
import re

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from xarm.wrapper import XArmAPI
from xarm.version import __version__

from doc.tool.markdown_doc import MarkdownDoc

open('../api/xarm_api.md', 'w', encoding='utf-8').write(
    pydoc.render_doc(XArmAPI,
                     title='xArm-Python-SDK API Documentation (V{}): %s'.format(__version__),
                     renderer=MarkdownDoc()))

# docs = pydoc.render_doc(XArmAPI,
#                         title='xArm-Python-SDK API Documentation: %s',
#                         renderer=MarkdownDoc()).split('\n')
# d = {'#': 1, '##': 2, '###': 3, '####': 4, '#####': 5, '######': 6}
# pattern = '#+\s'
#
#
# head_id = 0
# targetname = "../api/xarm_api.md"
# with open(targetname, 'w+') as f2:
#     for i in docs:
#         if not re.match(pattern, i.strip(' \t\n')):
#             continue
#         i = i.strip(' \t\n')
#         head = i.split(' ')[0]
#         f2.write('|' + '-----' * (len(head) - 1) + '@[' + i[len(head):].strip(' \t\n') + '](#id' + str(
#             head_id) + ')   \n')
#         head_id += 1
#     f2.write('\n')
#     head_id = 0
#     for i in docs:
#         if not re.match(pattern, i.strip(' \t\n')):
#             f2.write(i)
#             if '```' not in i:
#                 f2.write(' \t\n')
#         else:
#             i = i.strip(' \t\n')
#             head = i.split(' ')[0]
#             if head in d.keys():
#                 menu = ''.join(
#                     ['<h', str(len(head)), ' id=id', str(head_id), '>', i[len(head):].strip(' \t\n').replace('__', ''), '</h',
#                      str(len(head)), '>   \n'])
#                 f2.write(menu)
#                 head_id += 1
#
# print('done ...')

