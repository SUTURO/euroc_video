__author__ = 'tobi'

from voldemort_to_vader_data import SimulationRun
from tools import JsonTools
from voldemort_to_vader_data import TestContainer, SimulationRunContainer

class TestManager(object):
    OWL_PATH = "/home/suturo/catkin_ws/src/euroc/sr_experimental_data/"
    def __init__(self, mongo_tools):
        self.database_names = []
        self.simulation_runs = SimulationRunContainer()
        self.test_files = []
        self.tests = TestContainer()
        self.mongo_tools = mongo_tools

    def start(self):
        pass

    def get_simulation_run_by_name(self, name):
        return self.simulation_runs.get_run_by_name(name)

    def get_name_from_all_simulation_runs(self):
        sim_run_names = []
        for run_name in self.simulation_runs.runs:
            sim_run_names.append(run_name)
        return sim_run_names

    def add_all_tests_to_simulation_run(self, simulation_run):
        for test in self.tests.tests.itervalues():
            simulation_run.tests.add_test(test)

    def add_test_results_to_simulation_run(self,simulation_run, test_results):
        for test_result in test_results:
            simulation_run.tests.add_test_result_from_dict(test_result)
                        
    def read_tests_from_file(self, file_path):
        tests_data = JsonTools.parse_from_json_file(file_path)
        tests = TestContainer()
        tests_names = []

        for test_data in tests_data:
            self.tests.add_test_from_dict(test_data)
            tests_names.append(test_data['name'])
        return tests_names

    def read_simulation_runs(self):
        sim_run_names = self.mongo_tools.get_all_simulation_run_names_from_mongo()
        for name in sim_run_names:
            sim_run = SimulationRun(name)
            self.simulation_runs.add_run(sim_run)

    def transform_tests_to_ros_tests(self, tests):
        ros_test = []
        for test in tests:
            ros_test.append(test.to_ros_msg())
        return ros_test

    def get_topic_name_for_simulation_run(self, simulation_run_name):
        topic_names = self.mongo_tools.get_playable_topic_names_for_simulation_from_mongo(simulation_run_name)
        return topic_names
