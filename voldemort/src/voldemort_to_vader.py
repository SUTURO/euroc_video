#!/usr/bin/env python
__author__ = 'tobi'

from pymongo import MongoClient
import subprocess
import rospy
import json
import requests
import os
from suturo_video_msgs.srv import GetSimulationRuns, GetSimulationRunsRequest, GetSimulationRunsResponse, GetTests,\
    GetTestsRequest, GetTestsResponse, ExecuteTests, ExecuteTestsRequest, ExecuteTestsResponse, AddTests, \
    AddTestsRequest, AddTestsResponse
from suturo_video_msgs.msg import Test as RosTest
from suturo_video_msgs.msg import TestResult as RosTestResult
from voldemort_to_vader_data import Test, TestResult, SimulationRun, SimulationRunContainer, TestContainer

class VoldemortToVader(object):
    OWL_PATH = '/home/suturo/sr_experimental_data/'
    def __init__(self):
        self.client = MongoClient()
        self.database_names = []
        self.simulation_runs = SimulationRunContainer()
        self.test_files = []
        self.tests = TestContainer()

    def read_all_tests_and_results(self):
        self.get_all_simulation_runs_from_mongo()
        self.get_all_tests_for_all_runs()
        self.get_all_tests_global()
        self.get_all_test_results_for_all_runs()

    def get_all_test_results_for_all_runs(self):
        for name, sim_run in self.simulation_runs.runs.iteritems():
            self.get_all_test_results_from_mongo(sim_run)

    def start_services(self):
        rospy.init_node('voldemort_to_vader')
        get_simulation_runs_service = rospy.Service('/voldemort/get_simulation_runs', GetSimulationRuns, self.handle_get_simulation_runs)
        get_available_tests = rospy.Service('/voldemort/get_available_tests', GetTests, self.handle_get_available_tests)
        get_executed_tests = rospy.Service('/voldemort/get_executed_tests', GetTests, self.handle_get_executed_tests)
        get_failed_tests = rospy.Service('/voldemort/get_failed_tests', GetTests, self.handle_get_failed_tests)
        get_passed_tests = rospy.Service('/voldemort/get_passed_tests', GetTests, self.handle_get_passed_tests)
        execute_tests = rospy.Service('/voldemort/execute_tests', ExecuteTests, self.handle_execute_tests)
        add_tests = rospy.Service('/voldemort/add_tests', AddTests, self.handle_add_tests)
        print "[Voldemort_to_vader] Successfully started Services"
        rospy.spin()

    def handle_execute_tests(self, req):
        resp = ExecuteTestsResponse()
        simulation_run_name = None
        if req.simulation_run_name is not None:
            simulation_run_name = req.simulation_run_name
        elif req.simulation_run.name is not None:
            simulation_run_name = req.simulation_run.name

        if simulation_run_name:
            resp.result = self.execute_tests(simulation_run_name)
            resp.result = True
        else:
            resp.result = False
        return resp

    def handle_add_tests(self, req):
        resp = AddTestsResponse()
        resp.result = True
        tests_file_path = req.tests_file_path
        if os.path.isfile(tests_file_path):
            self.test_files.append(tests_file_path)
            self.read_all_tests_and_results()
            #with open(tests_file_path) as test_data:
             #   tests = test_data.readlines()
            tests = []
            for test_name in self.tests.tests:
                test = self.tests.get_test_by_name(test_name)
                tests.append(test.to_json_dict())
            self.upload_tests(tests)
        else:
            resp.result = False
        return resp


    def handle_get_simulation_runs(self, req):
        resp = GetSimulationRunsResponse()
        self.get_all_simulation_runs_from_mongo()
        sim_runs_list = []
        for run_name in self.simulation_runs.runs:
            sim_runs_list.append(run_name)

        resp.simulation_runs = sim_runs_list
        return resp

    def get_all_simulation_runs_from_mongo(self):
        database_names = self.client.database_names()
        for db_name in database_names:
            if db_name != 'local':
                sim_run = SimulationRun(db_name)
                self.simulation_runs.add_run(sim_run)

    def handle_get_available_tests(self, req):
        resp = GetTestsResponse()
        sim_run_name = req.simulation_run_name
        sim_run = self.simulation_runs.get_run_by_name(sim_run_name)
        self.get_all_test_results_from_mongo(sim_run)
        tests = []


        for name, test in sim_run.tests.iteritems():
            ros_test = RosTest()
            ros_test.name = name
            ros_test.description = test.description
            ros_test.query = test.query
            ros_test.expected = test.expected
            tests.append(ros_test)

        print "get_available_tests:"+str(sim_run.tests)

        resp.tests = tests
        return resp

    def handle_get_executed_tests(self, req):
        resp = GetTestsResponse()
        sim_run = self.simulation_runs.get_run_by_name(req.simulation_run_name)

        tests = []
        for name, test in sim_run.tests.iteritems():
            print(test.test_result)
            if test.test_result is not None:
                ros_test = RosTest()
                ros_test.name = name
                ros_test.description = test.description
                ros_test.query = test.query
                ros_test.expected = test.expected
                test_result = RosTestResult()
                test_result.result = test.test_result.result
                test_result.executionDate = test.test_result.execution_date
                print ('asdads'+str(test.test_result.notable_time_points))
                test_result.notableTimePoints = test.test_result.notable_time_points
                ros_test.test_result = test_result
                tests.append(ros_test)

        resp.tests = tests
        return resp

    def handle_get_failed_tests(self, req):
        resp = GetTestsResponse()
        sim_run = self.simulation_runs.get_run_by_name(req.simulation_run_name)

        tests = []
        for name, test in sim_run.tests.iteritems():
            if test.test_result is not None and not test.test_result.result:
                ros_test = RosTest()
                ros_test.name = name
                ros_test.description = test.description
                ros_test.query = test.query
                ros_test.expected = test.expected
                test_result = RosTestResult()
                test_result.result = test.test_result.result
                test_result.executionDate = test.test_result.execution_date
                test_result.notable_time_points = test.test_result.notable_time_points
                ros_test.test_result = test_result
                tests.append(ros_test)

        resp.tests = tests
        return resp

    def handle_get_passed_tests(self, req):
        resp = GetTestsResponse()
        sim_run = self.simulation_runs.get_run_by_name(req.simulation_run_name)

        tests = []
        for name, test in sim_run.tests.iteritems():
            if test.test_result is not None and test.test_result.result:
                ros_test = RosTest()
                ros_test.name = name
                ros_test.description = test.description
                ros_test.query = test.query
                ros_test.expected = test.expected
                test_result = RosTestResult()
                test_result.result = test.test_result.result
                test_result.executionDate = test.test_result.execution_date
                test_result.notable_time_points = test.test_result.notable_time_points
                ros_test.test_result = test_result
                tests.append(ros_test)

        resp.tests = tests
        return resp

    def get_all_tests_for_all_runs(self):
        for name, sim_run in self.simulation_runs.runs.iteritems():
            self.get_all_tests(sim_run)

    def get_all_tests(self, simulation_run):
        for test_file in self.test_files:
            self.get_all_tests_from_json(simulation_run, test_file)

    def get_all_tests_global(self):
        for test_file in self.test_files:
            self.get_all_tests_from_json_global(test_file)

    def get_all_tests_from_json(self, simulation_run, json_file):
        with open(json_file) as test_data:
            tests = json.load(test_data)
        for test_data in tests:
            test = Test(str(test_data['name']))
            test.query = str(test_data['query'])
            test.description = str(test_data['description'])
            test.expected = str(test_data['expected'])
            simulation_run.add_test(test)

    def get_all_tests_from_json_global(self, json_file):
        with open(json_file) as test_data:
            tests = json.load(test_data)
        for test_data in tests:
            test = Test(str(test_data['name']))
            test.query = str(test_data['query'])
            test.description = str(test_data['description'])
            test.expected = str(test_data['expected'])
            self.tests.add_test(test)

    def get_all_test_results_from_mongo(self, simulation_run):
        db = self.client[simulation_run.name]
        test_collection = db['test_results']
        for test_result_doc in test_collection.find():
            test_name = test_result_doc['name']
            test = simulation_run.get_test_by_name(test_name)
            if test is not None:
                test.test_result = self.create_test_result_from_mongo_document(test_result_doc)

    def create_test_result_from_mongo_document(self, test_result_doc):
        result = test_result_doc['result']
        execution_date = test_result_doc['executionDate']
        notable_time_points = test_result_doc['notableTimePoints']
        test_result = TestResult(result=result, execution_date=execution_date)
        for ntp_dict in notable_time_points:
            test_result.add_notable_time_point(ntp_dict)
        return test_result

    def handle_get_test_results(self, req):
        resp = GetTestResultsResponse()
        database_name = req.database_name
        test_results = self.get_test_results(database_name)
        resp.test_results = str(self.dump_test_results_to_json(test_results))
        return resp

    def get_test_results(self, simulation_run):
        test_db = self.client[self.TESTDB_NAME]
        for collection_name in test_db.collection_names(include_system_collections=False):
            if collection_name == simulation_run:
                return self.read_test_results_from_collection(test_db[collection_name])
        ## no tests found:
        self.start_tests(simulation_run)

    #TODO: fix Url
    def start_tests(self, database_name):
        url = "localhost"
        collection = self.TESTDB_NAME+'.'+database_name
        print collection
        robocop_caller_execute_cmd = 'rosrun voldemort call_robocop -u '+url+' -c'+collection
        robocop_caller = subprocess.Popen(robocop_caller_execute_cmd, stdout=subprocess.PIPE,
                                              shell=True)
        print "started_tests"

    ## only for testing purposes
    def delete_data(self, simulation_name):
        db = self.client[simulation_name]
        self.client.drop_database(simulation_name)

    def write_test_sim_db(self, simulation_name):
        db = self.client[simulation_name]
        simulation_data = {'sim': 'blubb'}
        db['test_collection'].insert_one(simulation_data)

    def write_test_results_from_file_to_mongo(self, simulation_name, test_results_filepath):
        with open(test_results_filepath) as test_results_data:
            test_results = json.load(test_results_data)
        self.write_test_results_to_mongo(simulation_name, test_results)

    def write_test_results_from_json_string_to_mongo(self, simulation_name, json_string):
        test_results = json.loads(json_string)
        self.write_test_results_to_mongo(simulation_name, test_results)

    def write_test_results_to_mongo(self, simulation_name, test_results):
        db = self.client[simulation_name]
        db.drop_collection('test_results')
        for test_result in test_results:
            db['test_results'].insert_one(test_result)

    def write_test_data(self):
        simulation_names = ['simulation1', 'simulation2', 'simulation3']
        for sim_name in simulation_names:
            vdv.delete_data(sim_name)
            vdv.write_test_sim_db(sim_name)

        test_file = 'test_data/tests2.json'
        test_results_file = 'test_data/tests2_results.json'
        vdv.write_test_results_from_file_to_mongo(simulation_names[0], test_results_file)

    def upload_tests(self, json_tests):
        params = {'test': str(json_tests)}
        r = requests.put('http://localhost:8080/robocop/uploadTest', data=params)

    def execute_tests(self, simulation_run_name):
        params = {'owl': str(self.OWL_PATH+simulation_run_name+'.owl'), 'db': simulation_run_name}
        r = requests.get('http://localhost:8080/robocop/executeTest', params=params)

if __name__ == '__main__':
    vdv = VoldemortToVader()

    #TODO:delete this lines when not testing
    vdv.write_test_data()
    #vdv.test_files.append('test_data/tests2.json')

    vdv.read_all_tests_and_results()
    vdv.start_services()


#TODO: tests nicht zu einer Simulation hinzufuegen, sondern global machen
#TODO: tests nicht aus json lesen sondern auch in die mongodb schreiben, damit tests und ergebnisse zusammen sind
#TODO: error handling for add_tests and execute_tests