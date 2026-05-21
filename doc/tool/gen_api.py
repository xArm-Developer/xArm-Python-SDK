#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, UFactory, Inc.
# All rights reserved.
#
# Author: Duke Fong <duke@ufactory.cc>


import sys
import os
import re
import pydoc

curr_file_dir = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(curr_file_dir, '../..'))

from xarm.wrapper import XArmAPI
from xarm.version import __version__

from doc.tool.markdown_doc import MarkdownDoc


def _make_anchor(heading_text):
    """Convert a markdown heading to a GitHub-style anchor fragment."""
    text = re.sub(r'^#+\s*', '', heading_text)
    anchor = re.sub(r'[^\w\- ]', '', text).lower().strip().replace(' ', '-')
    return anchor


def _extract_entries(content):
    """Parse generated markdown to extract methods and properties."""
    methods = []     # (name, anchor)
    properties = []  # (name, anchor)

    for line in content.split('\n'):
        stripped = line.strip()
        # Match: #### def __method_name__(self, ...params...):
        m = re.match(r'^####\s+def\s+__([a-zA-Z0-9_]+)__\(self,?\s*([^)]*)\)', stripped)
        if m:
            name = m.group(1)
            if name == 'init':
                continue
            anchor = _make_anchor(stripped)
            methods.append((name, anchor))
            continue

        # Match properties: #### property_name (not starting with "def")
        m = re.match(r'^####\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*$', stripped)
        if m and not stripped.startswith('#### def'):
            name = m.group(1)
            anchor = _make_anchor(stripped)
            properties.append((name, anchor))

    return methods, properties


def _build_toc(methods, properties):
    """Build a TOC markdown block."""
    lines = ['## Table of Contents', '']

    if methods:
        lines.append('### Methods ({})'.format(len(methods)))
        for name, anchor in methods:
            lines.append('- [{}](#{})'.format(name, anchor))
        lines.append('')

    if properties:
        lines.append('### Properties ({})'.format(len(properties)))
        for name, anchor in properties:
            lines.append('- [{}](#{})'.format(name, anchor))
        lines.append('')

    return '\n'.join(lines)


def _inject_toc(content, toc):
    """Insert TOC after the title line."""
    lines = content.split('\n')
    title_idx = 0
    for i, line in enumerate(lines):
        if line.startswith('xArm-Python-SDK') or line.startswith('# '):
            title_idx = i
            break
    result = lines[:title_idx + 1] + ['', toc, '---'] + lines[title_idx + 1:]
    return '\n'.join(result)


# 1. Generate markdown from XArmAPI class
doc_content = pydoc.render_doc(
    XArmAPI,
    title='xArm-Python-SDK API Documentation (V{}): %s'.format(__version__),
    renderer=MarkdownDoc()
)

# 2. Extract all API entries
methods, properties = _extract_entries(doc_content)

# Sort alphabetically
methods.sort(key=lambda x: x[0])
properties.sort(key=lambda x: x[0])

# 3. Build and inject TOC
toc = _build_toc(methods, properties)
doc_content = _inject_toc(doc_content, toc)

# 4. Write output
doc_filepath = os.path.join(curr_file_dir, '../api/xarm_api.md')
open(doc_filepath, 'w', encoding='utf-8').write(doc_content)

print('Generated: {}'.format(doc_filepath))
print('  Methods: {}'.format(len(methods)))
print('  Properties: {}'.format(len(properties)))
