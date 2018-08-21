#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>

from serial.tools import list_ports

def _dump_port(d):
    print('-' * 80)
    print('  device     : {}'.format(d.device))
    print('  hwid       : {}'.format(d.hwid))
    print('  product    : {}'.format(d.hwid))
    print('  description: {}'.format(d.hwid))
    print('-' * 80)

def get_ports(is_dump=True):
    ports = []
    for i in list_ports.comports():
        # if i.pid is not None and '{:04x}:{:04x}'.format(i.vid, i.pid) == vidpid:
        if i.pid is not None:
            if is_dump:
                _dump_port(i)
            ports.append({
                'pid': '{:04x}'.format(i.pid),
                'vid': '{:04x}'.format(i.vid),
                'device': i.device,
                'serial_number': i.serial_number,
                'hwid': i.hwid,
                'name': i.name,
                'description': i.description,
                'interface': i.interface,
                'location': i.location,
                'manufacturer': i.manufacturer,
                'product': i.product
            })
    return ports