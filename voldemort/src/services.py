__author__ = 'tobi'

import rospy
from suturo_video_msgs.srv import GetSimulationRuns, GetSimulationRunsRequest, GetSimulationRunsResponse, GetTests,\
    GetTestsRequest, GetTestsResponse, ExecuteTests, ExecuteTestsRequest, ExecuteTestsResponse, AddTests, \
    AddTestsRequest, AddTestsResponse
from voldemort_to_vader_data import Test, TestResult, SimulationRun, SimulationRunContainer, TestContainer
from tools import HttpTools
import os

class ServiceManager(object):
    def __init__(self, test_manager):
        self.test_manager = test_manager

    def start_services(self):
        get_simulation_runs_service = rospy.Service('/voldemort/get_simulation_runs', GetSimulationRuns, self.handle_get_simulation_runs)
        get_available_tests = rospy.Service('/voldemort/get_available_tests', GetTests, self.handle_get_available_tests)
        get_executed_tests = rospy.Service('/voldemort/get_executed_tests', GetTests, self.handle_get_executed_tests)
        get_failed_tests = rospy.Service('/voldemort/get_failed_tests', GetTests, self.handle_get_failed_tests)
        get_passed_tests = rospy.Service('/voldemort/get_passed_tests', GetTests, self.handle_get_passed_tests)
        execute_tests = rospy.Service('/voldemort/execute_tests', ExecuteTests, self.handle_execute_tests)
        add_tests = rospy.Service('/voldemort/add_tests', AddTests, self.handle_add_tests)
        print "[Voldemort_to_vader] Successfully started Services"

    def handle_execute_tests(self, req):
        resp = ExecuteTestsResponse()
        if req.simulation_run_name is not None:
            simulation_run_name = req.simulation_run_name
            resp.result = HttpTools.execute_tests(simulation_run_name)
            resp.result = True # Workaround
        if resp.result == True:
            sim_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
            # call a method from test_manager that turns the data from resp into a test_result (maybe from dict)
            self.test_manager.add_all_tests_to_simulation_run(sim_run)
        return resp

    def handle_add_tests(self, req):
        resp = AddTestsResponse()
        resp.result = True
        tests_file_path = req.tests_file_path
        if os.path.isfile(tests_file_path):
            test_names = self.test_manager.read_tests_from_file(tests_file_path)
            tests = []
            for test_name in test_names:
                test = self.test_manager.tests.get_test_by_name(test_name)
                tests.append(test.to_json_dict())
            self.upload_tests(tests)
        else:
            resp.result = False
        return resp

    def handle_get_simulation_runs(self, req):
        resp = GetSimulationRunsResponse()
        self.test_manager.read_simulation_runs()
        sim_run_names = self.test_manager.get_name_from_all_simulation_runs()
        resp.simulation_runs = sim_run_names
        return resp

    def handle_get_available_tests(self, req):
        resp = GetTestsResponse()
        tests = self.test_manager.tests.get_all_tests()
        ros_tests = self.test_manager.transform_tests_to_ros_tests(tests)
        resp.tests = ros_tests
        return resp

    def handle_get_executed_tests(self, req):
        resp = GetTestsResponse()
        simulation_run_name = req.simulation_run_name
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        tests = simulation_run.tests.get_executed_tests()
        resp.tests = self.test_manager.transform_tests_to_ros_tests(tests)
        print resp.tests
        return resp

    def handle_get_failed_tests(self, req):
        resp = GetTestsResponse()
        simulation_run_name = req.simulation_run_name
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        tests = simulation_run.tests.get_failed_tests()
        resp.tests = self.test_manager.transform_tests_to_ros_tests(tests)
        return resp

    def handle_get_passed_tests(self, req):
        resp = GetTestsResponse()
        simulation_run_name = req.simulation_run_name
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        tests = simulation_run.tests.get_passed_tests()
        resp.tests = self.test_manager.transform_tests_to_ros_tests(tests)
        return resp
