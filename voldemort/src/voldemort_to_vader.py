#!/usr/bin/env python
from services import ServiceManager
from test_manager import TestManager
from tools import MongoTools, TestDataTools

__author__ = 'tobi'

from pymongo import MongoClient
import subprocess
import rospy
import json
import requests
import os


class VoldemortToVader(object):
    def __init__(self):
        self.mongo_tools = None
        self.test_manager = None
        self.service_manager = None
        self.test_data_tool = None

    def start(self):
        self.mongo_tools = MongoTools()

        self.test_manager = TestManager(self.mongo_tools)
        self.service_manager = ServiceManager(self.test_manager)

        self.test_data_tool = TestDataTools(self.mongo_tools, self.test_manager)

        self.test_manager.start()
        self.test_data_tool.prepare_test_data()

        self.start_node()

    def start_node(self):
        rospy.init_node('voldemort_to_vader')
        self.service_manager.start_services()
        rospy.spin()

if __name__ == '__main__':
    vdv = VoldemortToVader()
    vdv.start()

#TODO:delete this lines when not testing

#TODO: tests nicht zu einer Simulation hinzufuegen, sondern global machen
#TODO: tests nicht aus json lesen sondern auch in die mongodb schreiben, damit tests und ergebnisse zusammen sind
#TODO: error handling for add_tests and execute_tests