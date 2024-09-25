#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32

from .interface_base import InterfaceBase


class DataInterface(InterfaceBase):

    def __init__(self, name: str = "unknown", rate: int = 10):
        super(DataInterface, self).__init__()

        ### ros node
        rospy.init_node(name, anonymous=True)
        self.__rate = rospy.Rate(rate)

        ### pamameter
        self._str_param = {
            "name": rospy.get_param('~str_name'),
        }
        self._int_param = {
            "range": rospy.get_param('~int_range'),
        }

        ### publisher
        self.__out_str_pub = rospy.Publisher('out_str', String, queue_size=10)
        self.__out_int_pub = rospy.Publisher('out_int', Int32, queue_size=10)

        ### subscriber
        self.__in_str_sub = rospy.Subscriber('in_str', String,
                                             self.__in_str_callback)
        self.__in_int_sub = rospy.Subscriber('in_int', Int32,
                                             self.__in_int_callback)
        self.__in_str_sub
        self.__in_int_sub

    def ok(self):
        return not rospy.is_shutdown()

    def sleep(self):
        self.__rate.sleep()

    def shutdown(self):
        pass

    def pub_out_str(self, out: str):
        msg = String()
        msg.data = out
        self.__out_str_pub.publish(msg)

    def pub_out_int(self, out: int):
        msg = Int32()
        msg.data = out
        self.__out_int_pub.publish(msg)

    def __in_str_callback(self, msg: String):
        with self._in_str_lock:
            self._in_str = msg.data
            self._in_str_flag = True

    def __in_int_callback(self, msg: Int32):
        with self._in_int_lock:
            self._in_int = msg.data
            self._in_int_flag = True
