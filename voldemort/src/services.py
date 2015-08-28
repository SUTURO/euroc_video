__author__ = 'tobi'

import rospy
from suturo_video_msgs.srv import GetSimulationRuns, GetSimulationRunsRequest, GetSimulationRunsResponse, GetTests,\
    GetTestsRequest, GetTestsResponse, ExecuteTests, ExecuteTestsRequest, ExecuteTestsResponse, AddTests, \
    AddTestsRequest, AddTestsResponse, GetTopicNames, GetTopicNamesRequest, GetTopicNamesResponse
from tools import HttpTools
from test_manager import TestManager
import os

class ServiceManager(object):
    def __init__(self, test_manager):
        self.test_manager = test_manager
        self.services = []

    def start_services(self):
        self.services.append(rospy.Service('/voldemort/get_simulation_runs', GetSimulationRuns, self.handle_get_simulation_runs))
        self.services.append(rospy.Service('/voldemort/get_available_tests', GetTests, self.handle_get_available_tests))
        self.services.append(rospy.Service('/voldemort/get_executed_tests', GetTests, self.handle_get_executed_tests))
        self.services.append(rospy.Service('/voldemort/get_failed_tests', GetTests, self.handle_get_failed_tests))
        self.services.append(rospy.Service('/voldemort/get_passed_tests', GetTests, self.handle_get_passed_tests))
        self.services.append(rospy.Service('/voldemort/execute_tests', ExecuteTests, self.handle_execute_tests))
        self.services.append(rospy.Service('/voldemort/add_tests', AddTests, self.handle_add_tests))
        self.services.append(rospy.Service('/voldemort/get_playable_topic_names', GetTopicNames, self.handle_get_playable_topic_names))
        print "[Voldemort_to_vader] Successfully started Services"

    def handle_execute_tests(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_executed_tests called"
        resp = ExecuteTestsResponse()
        resp.result = True
        simulation_run_name = req.simulation_run_name
        owl_file = TestManager.OWL_PATH + simulation_run_name + '/cram_log.owl'
        execute_result = HttpTools.execute_tests(simulation_run_name, owl_file)
        try:
            test_results = execute_result.json()
            print "[Voldemort_to_vader] Service: /voldemort/get_executed_tests Test_result="+str(test_results)

        except ValueError, e:
            resp.result = False
            print e

        if isinstance(test_results, dict):
            if test_results['result'] not in ['True', 'False']:
                print "[Voldemort_to_vader] Service: /voldemort/get_executed_tests Prolog Result Error=" +str(test_results)
                resp.result = False

        if execute_result.status_code != 200:
            resp.result = False

        if resp.result == True:
            sim_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
            # call a method from test_manager that turns the data from resp into a test_result (maybe from dict)
            self.test_manager.add_all_tests_to_simulation_run(sim_run)
            self.test_manager.add_test_results_to_simulation_run(sim_run, test_results)
            print "[Voldemort_to_vader] Service: /voldemort/get_executed_tests Result:True"
        else:
            print "[Voldemort_to_vader] Service: /voldemort/get_executed_tests Result:False"
        return resp

    def handle_add_tests(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/add_tests called"
        resp = AddTestsResponse()
        resp.result = True
        tests_file_path = req.tests_file_path
        if os.path.isfile(tests_file_path):
            test_names = self.test_manager.read_tests_from_file(tests_file_path)
            tests = []
            for test_name in test_names:

                test = self.test_manager.tests.get_test_by_name(test_name)
                tests.append(test.to_json_dict())
            upload_result = HttpTools.upload_tests(tests)
            if upload_result.status_code != 200:
                resp.result = False
        else:
            resp.result = False
        if resp.result == True:
            print "[Voldemort_to_vader] Service: /voldemort/add_tests Result:True"
        else:
            print "[Voldemort_to_vader] Service: /voldemort/add_tests Result:False"
        return resp

    def handle_get_simulation_runs(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_simulation_runs called"
        resp = GetSimulationRunsResponse()
        self.test_manager.read_simulation_runs()
        sim_run_names = self.test_manager.get_name_from_all_simulation_runs()
        resp.simulation_runs = sim_run_names
        return resp

    def handle_get_available_tests(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_available_tests called"
        resp = GetTestsResponse()
        tests = self.test_manager.tests.get_all_tests()
        ros_tests = self.test_manager.transform_tests_to_ros_tests(tests)
        resp.tests = ros_tests
        return resp

    def handle_get_executed_tests(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_executed_tests called"
        resp = GetTestsResponse()
        simulation_run_name = req.simulation_run_name
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        tests = simulation_run.tests.get_executed_tests()
        resp.tests = self.test_manager.transform_tests_to_ros_tests(tests)
        return resp

    def handle_get_failed_tests(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_failed_tests called"
        resp = GetTestsResponse()
        simulation_run_name = req.simulation_run_name
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        tests = simulation_run.tests.get_failed_tests()
        resp.tests = self.test_manager.transform_tests_to_ros_tests(tests)
        return resp

    def handle_get_passed_tests(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_passed_tests called"
        resp = GetTestsResponse()
        simulation_run_name = req.simulation_run_name
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        tests = simulation_run.tests.get_passed_tests()
        resp.tests = self.test_manager.transform_tests_to_ros_tests(tests)
        return resp

    def handle_get_playable_topic_names(self, req):
        print "[Voldemort_to_vader] Service: /voldemort/get_playable_topic_names called"
        resp = GetTopicNamesResponse()
        simulation_name = req.simulation_run_name
        topic_names = self.test_manager.get_topic_name_for_simulation_run(simulation_name)
        resp.topic_names = topic_names
        return resp