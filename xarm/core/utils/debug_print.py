#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./common_modules/debug_print.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#


def print_hex(str, data, len):
    for i in range(len):
        str += ('%0.2X ' % data[i])
    print(str)


def print_nvect(str, data, len):
    for i in range(len):
        str += ('%0.3f ' % data[i])
    print(str)
