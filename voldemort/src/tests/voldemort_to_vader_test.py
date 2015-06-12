__author__ = 'tobi'

import unittest
from voldemort_to_vader import VoldemortToVader
from voldemort_to_vader_data import SimulationRun, Test, TestResult
from suturo_video_msgs.srv import GetSimulationRunsRequest, GetSimulationRunsResponse, GetTestsRequest, GetTestsResponse
from suturo_video_msgs.msg import Test as RosTest

class VoldemortToVaderTest(unittest.TestCase):
    def test_get_all_simulation_runs_from_mongo(self):
        vdv = VoldemortToVader()

        simulation_names = ['simulation1', 'simulation2', 'simulation3']
        for sim_name in simulation_names:
            vdv.write_test_sim_db(sim_name)

        vdv.get_all_simulation_runs_from_mongo()

        for sim_name in simulation_names:
            sim_run = vdv.simulation_runs.get_run_by_name(sim_name)
            self.assertEqual(sim_run.name, sim_name)

    def test_get_all_tests_from_json(self):
        simulation_name = 'simulation1'
        test_file = 'test_data/tests2.json'

        vdv = VoldemortToVader()
        actual_simulation_run = SimulationRun(simulation_name)

        vdv.delete_data(simulation_name)
        vdv.write_test_sim_db(simulation_name)

        vdv.simulation_runs.add_run(actual_simulation_run)
        vdv.get_all_tests_from_json(actual_simulation_run, test_file)

        simulation_run = vdv.simulation_runs.get_run_by_name(simulation_name)

        self.assertTrue(simulation_run)
        self.assertEqual(len(simulation_run.tests), 2)

        test1 = simulation_run.get_test_by_name('test_name1')
        test2 = simulation_run.get_test_by_name('test_name2')

        self.assertEqual(test1.name, 'test_name1')
        self.assertEqual(test1.query, 'test_query1')
        self.assertEqual(test1.description, 'test_description1')
        self.assertEqual(test2.description, 'test_description2')

    def test_get_all_test_results_from_mongo(self):
        simulation_name = 'simulation1'
        test_results_file = 'test_data/tests2_results.json'

        vdv = VoldemortToVader()
        actual_simulation_run = SimulationRun(simulation_name)

        vdv.delete_data(simulation_name)
        vdv.write_test_results_to_mongo(simulation_name, test_results_file)

        test1 = Test("test_name1")
        test2 = Test("test_name2")
        actual_simulation_run.add_test(test1)
        actual_simulation_run.add_test(test2)
        vdv.simulation_runs.add_run(actual_simulation_run)

        vdv.get_all_test_results_from_mongo(actual_simulation_run)

        self.assertEqual(test1.test_result.result, True)
        self.assertEqual(test1.test_result.execution_date, "2015-05-06_16:26:02")
        self.assertEqual(test2.test_result.result, False)
        self.assertEqual(test2.test_result.execution_date, "2011-05-86_26:16:01")


    def test_handle_get_simulation_runs(self):
        vdv = VoldemortToVader()

        simulation_names = ['simulation1', 'simulation2', 'simulation3']
        for sim_name in simulation_names:
            vdv.write_test_sim_db(sim_name)

        req = GetSimulationRunsRequest()
        response = vdv.handle_get_simulation_runs(req)
        runs = response.simulation_runs

        for name in simulation_names:
            self.assertIn(name, runs)

    def test_handle_get_available_tests(self):
        vdv = VoldemortToVader()

        simulation_name = 'simulation1'
        test_file = 'test_data/tests2.json'
        test_results_file = 'test_data/tests2_results.json'

        vdv.delete_data(simulation_name)
        vdv.write_test_sim_db(simulation_name)

        actual_simulation_run = SimulationRun(simulation_name)

        vdv.simulation_runs.add_run(actual_simulation_run)

        req = GetTestsRequest()
        req.simulation_run_name = simulation_name

        resp = vdv.handle_get_available_tests(req)
        tests = resp.tests
        self.assertEqual(len(tests), 2)
        test1 = tests[0]
        test2 = tests[1]
        self.assertEqual(test1.name, 'test_name1')
        self.assertEqual(test1.description, 'test_description1')
        self.assertEqual(test1.query, 'test_query1')

        self.assertEqual(test2.name, 'test_name2')
        self.assertEqual(test2.description, 'test_description2')
        self.assertEqual(test2.query, 'test_query2')