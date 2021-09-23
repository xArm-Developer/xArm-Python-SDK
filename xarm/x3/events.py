#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

REPORT_ID = 'REPORT'
REPORT_LOCATION_ID = 'LOCATION'
REPORT_CONNECT_CHANGED_ID = 'REPORT_CONNECT_CHANGED'
REPORT_STATE_CHANGED_ID = 'REPORT_STATE_CHANGED'
REPORT_MODE_CHANGED_ID = 'REPORT_MODE_CHANGED'
REPORT_MTABLE_MTBRAKE_CHANGED_ID = 'REPORT_MTABLE_MTBRAKE_CHANGED'
REPORT_ERROR_WARN_CHANGED_ID = 'REPORT_ERROR_WARN_CHANGED'
REPORT_CMDNUM_CHANGED_ID = 'REPORT_CMDNUM_CHANGED'
REPORT_TEMPERATURE_CHANGED_ID = 'REPORT_TEMPERATURE_CHANGED'
REPORT_COUNT_CHANGED_ID = 'REPORT_COUNT_CHANGED'
REPORT_IDEN_PROGRESS_CHANGED_ID = 'REPORT_IDEN_PROGRESS_CHANGED_ID'


class Events(object):
    REPORT_ID = REPORT_ID
    REPORT_LOCATION_ID = REPORT_LOCATION_ID
    REPORT_CONNECT_CHANGED_ID = REPORT_CONNECT_CHANGED_ID
    REPORT_STATE_CHANGED_ID = REPORT_STATE_CHANGED_ID
    REPORT_MODE_CHANGED_ID = REPORT_MODE_CHANGED_ID
    REPORT_MTABLE_MTBRAKE_CHANGED_ID = REPORT_MTABLE_MTBRAKE_CHANGED_ID
    REPORT_ERROR_WARN_CHANGED_ID = REPORT_ERROR_WARN_CHANGED_ID
    REPORT_CMDNUM_CHANGED_ID = REPORT_CMDNUM_CHANGED_ID
    REPORT_TEMPERATURE_CHANGED_ID = REPORT_TEMPERATURE_CHANGED_ID
    REPORT_COUNT_CHANGED_ID = REPORT_COUNT_CHANGED_ID
    REPORT_IDEN_PROGRESS_CHANGED_ID = REPORT_IDEN_PROGRESS_CHANGED_ID

    def __init__(self):
        self._report_callbacks = {
            REPORT_ID: [],
            REPORT_LOCATION_ID: [],
            REPORT_CONNECT_CHANGED_ID: [],
            REPORT_ERROR_WARN_CHANGED_ID: [],
            REPORT_STATE_CHANGED_ID: [],
            REPORT_MODE_CHANGED_ID: [],
            REPORT_MTABLE_MTBRAKE_CHANGED_ID: [],
            REPORT_CMDNUM_CHANGED_ID: [],
            REPORT_COUNT_CHANGED_ID: [],
            REPORT_IDEN_PROGRESS_CHANGED_ID: []
        }

    def _register_report_callback(self, report_id, callback):
        if report_id not in self._report_callbacks.keys():
            self._report_callbacks[report_id] = []
        if (callable(callback) or isinstance(callback, dict)) and callback not in self._report_callbacks[report_id]:
            self._report_callbacks[report_id].append(callback)
            return True
        elif not (callable(callback) or isinstance(callback, dict)):
            return False
        else:
            return True

    def _release_report_callback(self, report_id, callback):
        if report_id in self._report_callbacks.keys():
            if callback is None:
                self._report_callbacks[report_id].clear()
                return True
            elif callback:
                for cb in self._report_callbacks[report_id]:
                    if callback == cb:
                        self._report_callbacks[report_id].remove(callback)
                        return True
                    elif isinstance(cb, dict):
                        if cb['callback'] == callback:
                            self._report_callbacks[report_id].remove(cb)
                            return True
        return False

    def register_report_callback(self, callback=None, report_cartesian=True, report_joints=True,
                                 report_state=True, report_error_code=True, report_warn_code=True,
                                 report_mtable=True, report_mtbrake=True, report_cmd_num=True):
        return self._register_report_callback(REPORT_ID, {
            'callback': callback,
            'cartesian': report_cartesian,
            'joints': report_joints,
            'error_code': report_error_code,
            'warn_code': report_warn_code,
            'state': report_state,
            'mtable': report_mtable,
            'mtbrake': report_mtbrake,
            'cmdnum': report_cmd_num
        })

    def register_report_location_callback(self, callback=None, report_cartesian=True, report_joints=False):
        ret = self._register_report_callback(REPORT_LOCATION_ID, {
            'callback': callback,
            'cartesian': report_cartesian,
            'joints': report_joints,
        })
        return ret

    def register_connect_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_CONNECT_CHANGED_ID, callback)

    def register_state_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_STATE_CHANGED_ID, callback)

    def register_mode_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_MODE_CHANGED_ID, callback)

    def register_mtable_mtbrake_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_MTABLE_MTBRAKE_CHANGED_ID, callback)

    def register_error_warn_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_ERROR_WARN_CHANGED_ID, callback)

    def register_cmdnum_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_CMDNUM_CHANGED_ID, callback)

    def register_temperature_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_TEMPERATURE_CHANGED_ID, callback)

    def register_count_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_COUNT_CHANGED_ID, callback)

    def register_iden_progress_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_IDEN_PROGRESS_CHANGED_ID, callback)

    def release_report_callback(self, callback=None):
        return self._release_report_callback(REPORT_ID, callback)

    def release_report_location_callback(self, callback=None):
        return self._release_report_callback(REPORT_LOCATION_ID, callback)

    def release_connect_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_CONNECT_CHANGED_ID, callback)

    def release_state_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_STATE_CHANGED_ID, callback)

    def release_mode_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_MODE_CHANGED_ID, callback)

    def release_mtable_mtbrake_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_MTABLE_MTBRAKE_CHANGED_ID, callback)

    def release_error_warn_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_ERROR_WARN_CHANGED_ID, callback)

    def release_cmdnum_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_CMDNUM_CHANGED_ID, callback)

    def release_temperature_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_TEMPERATURE_CHANGED_ID, callback)

    def release_count_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_COUNT_CHANGED_ID, callback)

    def release_iden_progress_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_IDEN_PROGRESS_CHANGED_ID, callback)
