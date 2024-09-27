#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import queue
import typing
from abc import ABC, abstractmethod


class InterfaceBase(ABC):

    def __init__(self):
        ### ROS parameters
        self._str_param = {}
        self._int_param = {}

        ### ROS RX msg queues
        self._in_str_queue = queue.Queue()
        self._in_int_queue = queue.Queue()

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
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")
    
    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")
    
    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")
    
    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")
    
    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")

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
        return self._in_str_queue.qsize() > 0

    def clear_in_str(self):
        self._in_str_queue.queue.clear()

    def get_in_str(self) -> typing.Optional[str]:
        try:
            return self._in_str_queue.get_nowait()
        except queue.Empty:
            return None

    # in int
    def has_in_int(self) -> bool:
        return self._in_int_queue.qsize() > 0

    def clear_in_int(self):
        self._in_int_queue.queue.clear()

    def get_in_int(self) -> typing.Optional[int]:
        try:
            return self._in_int_queue.get_nowait()
        except queue.Empty:
            return None
