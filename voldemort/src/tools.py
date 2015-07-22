__author__ = 'tobi'

import json
import requests
from pymongo import MongoClient

class MongoTools(object):
    def __init__(self):
        self.client = MongoClient()

    def write_data_to_mongo_db(self, db_name, collection_name, data):
        db = self.client[db_name]
        db[collection_name].insert_one(data)

    def write_test_simulation_data_to_mongo(self, simulation_name):
        test_collection_name = 'test_collection'
        simulation_data = {'sim': 'blubb'}
        self.write_data_to_mongo_db(simulation_name, test_collection_name, simulation_data)

    def drop_database(self, database_name):
        self.client.drop_database(database_name)

    def write_test_results_to_mongo(self, simulation_name, test_results):
        db = self.client[simulation_name]
        db.drop_collection('test_results')
        for test_result in test_results:
            db['test_results'].insert_one(test_result)

    def create_test_result_from_mongo_document(self, test_result_doc):
        result = test_result_doc['result']
        execution_date = test_result_doc['executionDate']
        notable_time_points = test_result_doc['notableTimePoints']
        test_result = TestResult(result=result, execution_date=execution_date)
        for ntp_dict in notable_time_points:
            test_result.add_notable_time_point(ntp_dict)
        return test_result

    def get_all_test_results_from_mongo(self, simulation_run):
        db = self.client[simulation_run.name]
        test_collection = db['test_results']
        for test_result_doc in test_collection.find():
            test_name = test_result_doc['name']
            test = simulation_run.get_test_by_name(test_name)
            if test is not None:
                test.test_result = self.create_test_result_from_mongo_document(test_result_doc)

    def get_all_simulation_run_names_from_mongo(self):
        database_names = self.client.database_names()
        if 'local' in database_names:
            database_names.remove('local')
        return database_names

class JsonTools(object):
    @staticmethod
    def parse_from_json_file(file_path):
        with open(file_path) as test_data:
            test_data = json.load(test_data)
        return test_data


class HttpTools(object):
    OWL_PATH = '/home/suturo/sr_experimental_data/'

    @staticmethod
    def upload_tests(self, json_tests):
        params = {'test': str(json_tests)}
        r = requests.put('http://localhost:8080/robocop/uploadTest', data=params)

    @staticmethod
    def execute_tests(self, simulation_run_name):
        params = {'owl': str(self.OWL_PATH+simulation_run_name+'.owl'), 'db': simulation_run_name}
        r = requests.get('http://localhost:8080/robocop/executeTest', params=params)


class TestDataTools(object):
    def __init__(self, mongo_tools, test_manager):
        self.mongo_tools = mongo_tools
        self.test_manager = test_manager
        self.simulation_names = ['simulation1', 'simulation2', 'simulation3']
        self.test_result_files = ['test_data/tests1_results.json', 'test_data/tests2_results.json', 'test_data/tests3_results.json']
        self.test_file = 'test_data/tests.json'


    def prepare_test_data(self):
        self.write_test_simulation_dbs()
        self.read_tests()
        self.read_test_results()

    def write_test_simulation_dbs(self):
        for name in self.simulation_names:
            self.mongo_tools.write_test_simulation_data_to_mongo(name)

    def read_tests(self):
        self.test_manager.read_tests_from_file(self.test_file)
        for sim_run_name in self.simulation_names:
            sim_run = self.test_manager.get_simulation_run_by_name(sim_run_name)
            self.test_manager.add_all_tests_to_simulation_run(sim_run)

    def read_test_results(self):
        for index, file_path in enumerate(self.test_result_files):
            self.read_test_results_from_file(self.simulation_names[index], self.test_result_files[index])

    def read_test_results_from_file(self, simulation_run_name, file_path):
        simulation_run = self.test_manager.get_simulation_run_by_name(simulation_run_name)
        test_results = JsonTools.parse_from_json_file(file_path)
        for test_result in test_results:
            simulation_run.tests.add_test_result_from_dict(test_result)

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