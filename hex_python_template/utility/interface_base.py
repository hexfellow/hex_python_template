#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import threading
from abc import ABC, abstractmethod


class InterfaceBase(ABC):

    def __init__(self):
        ### parameters
        self._str_param = {}
        self._int_param = {}

        ### subscribers
        self._in_str_flag = False
        self._in_int_flag = False
        self._in_str = None
        self._in_int = None

        ### locks
        self._in_str_lock = threading.Lock()
        self._in_int_lock = threading.Lock()

    def __del__(self):
        self.shutdown()

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    ####################
    ### parameters
    ####################
    def get_str_param(self) -> dict:
        return self._str_param

    def get_int_param(self) -> dict:
        return self._int_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_out_str(self, out: str):
        raise NotImplementedError("InterfaceBase.pub_out_str")

    @abstractmethod
    def pub_out_int(self, out: int):
        raise NotImplementedError("InterfaceBase.pub_out_int")

    ####################
    ### subscribers
    ####################
    # in str
    def has_in_str(self) -> bool:
        return self._in_str_flag

    def clear_in_str(self):
        with self._in_str_lock:
            self._in_str_flag = False

    def get_in_str(self) -> str:
        with self._in_str_lock:
            return self._in_str

    # in int
    def has_in_int(self) -> bool:
        return self._in_int_flag

    def clear_in_int(self):
        with self._in_int_lock:
            self._in_int_flag = False

    def get_in_int(self) -> int:
        with self._in_int_lock:
            return self._in_int
