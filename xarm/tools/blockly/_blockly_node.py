#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET
import re


class _BlocklyNode(object):
    def __init__(self, xml_path):
        self._root = ET.parse(xml_path).getroot()
        self._ns = self.__get_ns()
    
    def __get_ns(self):
        try:
            r = re.compile('({.+})')
            if r.search(self._root.tag) is not None:
                ns = r.search(self._root.tag).group(1)
            else:
                ns = ''
        except Exception as e:
            print('get namespace exception: {}'.format(e))
            ns = ''
        return ns

    def _get_node(self, tag, root=None):
        root = self._root if root is None else root
        return root.find(self._ns + tag)

    def _get_nodes(self, tag, root=None, descendant=False, **kwargs):
        root = self._root if root is None else root
        nodes = []
        func = root.iter if descendant else root.findall
        for node in func(self._ns + tag):
            flag = True
            for k, v in kwargs.items():
                if node.attrib[k] != v:
                    flag = False
            if flag:
                nodes.append(node)
        return nodes

    def get_node(self, tag, root=None):
        """
        Only call in studio
        """
        return self._get_node(tag, root=root)
    
    def get_nodes(self, tag, root=None, descendant=False, **kwargs):
        """
        Only call in studio
        """
        return self._get_nodes(tag, root=root, descendant=descendant, **kwargs)
 