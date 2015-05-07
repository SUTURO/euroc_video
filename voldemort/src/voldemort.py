#!/usr/bin/env python
__author__ = 'tobi'

from pymongo import MongoClient
import subprocess
import rospy
import json
from suturo_video_msgs.srv import GetSimulationRuns, GetTestResults, GetSimulationRunsRequest, GetSimulationRunsResponse, GetTestResultsResponse, GetTestResultsRequest

class Voldemort(object):
    def __init__(self):
        self.client = MongoClient()
        self.TESTDB_NAME = 'robocop_test_results'
        self.database_names = []

    def start_services(self):
        rospy.init_node('voldemort')
        get_simulation_runs_service = rospy.Service('/voldemort/get_simulation_runs', GetSimulationRuns, self.handle_get_simulation_runs)
        get_test_results = rospy.Service('/voldemort/get_test_results', GetTestResults, self.handle_get_test_results)
        rospy.spin()

    def handle_get_simulation_runs(self, req):
        resp = GetSimulationRunsResponse()
        simulation_runs = self.get_simulation_runs()
        resp.database_names = simulation_runs
        return resp

    def get_simulation_runs(self):
        simulation_runs = []
        database_names = self.client.database_names()
        for db_name in database_names:
            if db_name != 'local' and db_name != self.TESTDB_NAME:
                simulation_runs.append(db_name)
        return simulation_runs

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

    #TODO: adjust when test data is available
    def read_test_results_from_collection(self, collection):
        # returns the first collection (if there are more it has to be adjusted)
        test_results = collection.find_one()
        return test_results

    #TODO: adjust when test data is available
    def dump_test_results_to_json(self, test_results):
        if test_results is not None and test_results['_id'] is not None:
            del test_results['_id']
        return json.dumps(test_results)

    ## only for testing purposes
    def write_test_data(self):
        tests = [{'name':'TESTNAME1'}, {'name': 'TESTNAME2'}]
        test_data = {'tests': tests}

        db = self.client[self.TESTDB_NAME]
        db.drop_collection('07-05-2015-02-50-20')
        db['07-05-2015-02-50-20'].insert_one(test_data)

if __name__ == '__main__':
    voldemort = Voldemort()
    #TODO:delete this line when not testing
    voldemort.write_test_data()

    voldemort.start_services()
