__author__ = 'tobi'

import unittest
from voldemort_to_vader import VoldemortToVader
from voldemort_to_vader_data import SimulationRun, Test, TestResult
from suturo_video_msgs.srv import GetSimulationRunsRequest, GetSimulationRunsResponse, GetTestsRequest, GetTestsResponse
from suturo_video_msgs.msg import Test as RosTest

class VoldemortToVaderTest(unittest.TestCase):
    def setUp(self):
        self.vdv = VoldemortToVader()
        self.simulation_name = 'simulation1'
        test_results_file = 'test_data/tests2_results.json'

        self.vdv.delete_data(self.simulation_name)
        self.vdv.write_test_sim_db(self.simulation_name)
        self.vdv.write_test_results_from_file_to_mongo(self.simulation_name, test_results_file)

        self.vdv.test_files.append('test_data/tests2.json')
        self.vdv.read_all_tests_and_results()

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
        self.assertEqual(len(simulation_run.tests), 5)

        test1 = simulation_run.get_test_by_name('test_name1')
        test4 = simulation_run.get_test_by_name('test_name4')

        self.assertEqual(test1.name, 'test_name1')
        self.assertEqual(test1.query, 'test_query1')
        self.assertEqual(test1.description, 'test_description1')
        self.assertEqual(test4.description, 'test_description4')

    def test_get_all_test_results_from_mongo(self):
        simulation_name = 'simulation1'
        test_results_file = 'test_data/tests2_results.json'

        vdv = VoldemortToVader()
        actual_simulation_run = SimulationRun(simulation_name)

        vdv.delete_data(simulation_name)
        vdv.write_test_results_from_file_to_mongo(simulation_name, test_results_file)

        test1 = Test("test_name1")
        test2 = Test("test_name2")
        test3 = Test("test_name3")
        test4 = Test("test_name4")
        test5 = Test("test_name5")
        tests = [test1, test2, test3, test4, test5]

        for test in tests:
            actual_simulation_run.add_test(test)
        vdv.simulation_runs.add_run(actual_simulation_run)

        vdv.get_all_test_results_from_mongo(actual_simulation_run)

        self.assertEqual(test1.test_result.result, True)
        self.assertEqual(test1.test_result.execution_date, "2015-05-06_16:26:02")
        self.assertEqual(test4.test_result.result, False)
        self.assertEqual(test4.test_result.execution_date, "2015-02-36_46:17:03")


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

        vdv.delete_data(simulation_name)
        vdv.write_test_sim_db(simulation_name)

        actual_simulation_run = SimulationRun(simulation_name)

        vdv.simulation_runs.add_run(actual_simulation_run)

        req = GetTestsRequest()
        req.simulation_run_name = simulation_name

        resp = vdv.handle_get_available_tests(req)
        tests = resp.tests
        self.assertEqual(len(tests), 5)
        test1 = tests[0]
        test4 = tests[3]
        self.assertEqual(test1.name, 'test_name1')
        self.assertEqual(test1.description, 'test_description1')
        self.assertEqual(test1.query, 'test_query1')

        self.assertEqual(test4.name, 'test_name4')
        self.assertEqual(test4.description, 'test_description4')
        self.assertEqual(test4.query, 'test_query4')

    def test_handle_get_executed_tests(self):
        vdv = self.vdv

        simulation_run = vdv.simulation_runs.get_run_by_name(self.simulation_name)

        req = GetTestsRequest()
        req.simulation_run_name = self.simulation_name

        resp = vdv.handle_get_executed_tests(req)
        tests = resp.tests
        self.assertEqual(len(tests), 4)
        expectedTestNames = ['test_name1', 'test_name2', 'test_name4', 'test_name5']
        for index, test in enumerate(tests):
            self.assertTrue(test.name in expectedTestNames)

        self.assertTrue(tests[0].test_result.result)
        self.assertFalse(tests[1].test_result.result)
        self.assertFalse(tests[2].test_result.result)
        self.assertTrue(tests[3].test_result.result)

        self.assertEqual(tests[0].test_result.execution_date, '2015-05-06_16:26:02')
        self.assertEqual(tests[1].test_result.execution_date, '2011-05-86_26:16:01')
        self.assertEqual(tests[2].test_result.execution_date, '2015-02-36_46:17:03')
        self.assertEqual(tests[3].test_result.execution_date, '2012-02-36_46:17:02')

    def test_handle_get_passed_tests(self):
        vdv = self.vdv

        simulation_run = vdv.simulation_runs.get_run_by_name(self.simulation_name)

        req = GetTestsRequest()
        req.simulation_run_name = self.simulation_name

        resp = vdv.handle_get_passed_tests(req)
        tests = resp.tests
        self.assertEqual(len(tests), 2)
        expectedTestNames = ['test_name1', 'test_name5']
        for index, test in enumerate(tests):
            self.assertTrue(test.name in expectedTestNames)
            self.assertTrue(test.test_result.result)

        self.assertEqual(tests[0].test_result.execution_date, '2015-05-06_16:26:02')
        self.assertEqual(tests[1].test_result.execution_date, '2012-02-36_46:17:02')

    def test_handle_get_failed_tests(self):
        vdv = self.vdv

        simulation_run = vdv.simulation_runs.get_run_by_name(self.simulation_name)

        req = GetTestsRequest()
        req.simulation_run_name = self.simulation_name

        resp = vdv.handle_get_failed_tests(req)
        tests = resp.tests
        self.assertEqual(len(tests), 2)
        expectedTestNames = ['test_name2', 'test_name4']
        for index, test in enumerate(tests):
            self.assertTrue(test.name in expectedTestNames)
            self.assertFalse(test.test_result.result)

        self.assertEqual(tests[0].test_result.execution_date, '2011-05-86_26:16:01')
        self.assertEqual(tests[1].test_result.execution_date, '2015-02-36_46:17:03')


    #TODO: Test mit Test Results ohne dazugehoerigem Test
    #TODO: Test wenn sich der Test aendert(also zB der Query, aber nicht der Name) sollte wohl das Testergebnis geloescht werden
    # oder zwischen den Tests unterschieden werden