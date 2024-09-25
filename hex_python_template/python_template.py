#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import os
import sys

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface


class PythonTemplate:

    def __init__(self):
        ### utility, Create a node called python_template and set the rate to 20Hz
        self.__data_interface = DataInterface("python_template", 20)

        ### parameters
        # This demos how to get parameters from the ROS
        self.__str_param = self.__data_interface.get_str_param()
        self.__int_param = self.__data_interface.get_int_param()

        ### variables
        self.__out_str = None
        self.__out_int = None

    def run(self):
        while self.__data_interface.ok():
            # Get data from ros every wake up
            while self.__data_interface.has_in_str():
                in_str = self.__data_interface.get_in_str()
                print(f"in_str: {in_str}")
                self.__data_interface.clear_in_str()
                self.__out_str = f"{self.__str_param['name']}: {in_str}"

            while self.__data_interface.has_in_int():
                in_int = self.__data_interface.get_in_int()
                print(f"in_int: {in_int}")
                self.__data_interface.clear_in_int()
                self.__out_int = in_int if in_int > self.__int_param['range'][
                    0] else self.__int_param['range'][0]
                self.__out_int = self.__out_int if self.__out_int < self.__int_param[
                    'range'][1] else self.__int_param['range'][1]

            if self.__out_str is not None:
                print(f"out_str: {self.__out_str}")
                self.__data_interface.pub_out_str(self.__out_str)

            if self.__out_int is not None:
                print(f"out_int: {self.__out_int}")
                self.__data_interface.pub_out_int(self.__out_int)

            self.__data_interface.sleep()


def main():
    python_template = PythonTemplate()
    python_template.run()


if __name__ == '__main__':
    main()
