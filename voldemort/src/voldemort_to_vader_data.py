__author__ = 'tobi'

import rospy
import copy
from suturo_video_msgs.msg import Test as RosTest
from suturo_video_msgs.msg import Binding as RosBinding
from suturo_video_msgs.msg import TestResult as RosTestResult

class SimulationRun(object):
    def __init__(self, name):
        self.name = name
        self.tests = TestContainer()

class SimulationRunContainer(object):
    def __init__(self):
        self.runs = {}

    def add_run(self, run):
        if not self.get_run_by_name(run.name):
            self.runs[run.name] = run

    def get_run_by_name(self, name):
        run = None
        try:
            run = self.runs[name]
        except KeyError:
            pass
        return run

class TestContainer(object):
    def __init__(self):
        self.tests = {}

    def add_test(self, test):
        self.tests[test.name] = copy.deepcopy(test) # deepcopy, so that all tests and those of each SimulationRun dont reference the same test objects

    def add_test_from_dict(self, test_data):
        name = test_data['name']
        test = Test(name)
        test.description = test_data['description']
        test.query = test_data['query']
        expected = test_data['expected']
        if isinstance(expected, dict):
            new_expected = []
            for key, value in expected.iteritems():
                tmp_dict = {}
                tmp_dict[key] = value
                new_expected.append(tmp_dict)
            expected = new_expected
        test.expected = expected

        self.tests[name] = test
        return name

    def add_test_result_from_dict(self, test_result_data):
        test_result = None
        test_result = TestResult()
        test_name = test_result_data['name']

        test_result.execution_date = test_result_data['executionDate']
        test_result.result = test_result_data['result']
        bindings = test_result_data['bindings']
        if isinstance(bindings, dict):
            new_bindings = []
            for key, value in bindings.iteritems():
                tmp_dict = {}
                tmp_dict[key] = value
                new_bindings.append(tmp_dict)
            bindings = new_bindings

        test_result.bindings = bindings
        for time_point in test_result_data['notableTimePoints']:
            test_result.add_notable_time_point(time_point)

        #self.time_point_no_dirty_hack_hahaha(test_result, test_name)

        test = self.get_test_by_name(test_name)
        test.test_result = test_result

    def time_point_no_dirty_hack_hahaha(self, test_result, test_name):
        if test_name == "AllObjectsRecognized":
            test_result.add_notable_time_point_by_values(116, 208)
            test_result.add_notable_time_point_by_values(147, 280 )
            test_result.add_notable_time_point_by_values(170, 307)
            test_result.add_notable_time_point_by_values(188, 896)
        elif test_name == "AllPlacedObjects":
            test_result.add_notable_time_point_by_values(116, 208)
            test_result.add_notable_time_point_by_values(147, 280)
            test_result.add_notable_time_point_by_values(170, 307)
            test_result.add_notable_time_point_by_values(188, 896)
        elif test_name == "AllGraspedObjects":
            test_result.add_notable_time_point_by_values(91, 937)
            test_result.add_notable_time_point_by_values(115, 871)
            test_result.add_notable_time_point_by_values(148, 256)
            test_result.add_notable_time_point_by_values(170, 047)


    def get_test_by_name(self, name):
        test = None
        try:
            test = self.tests[name]
        except KeyError:
            print("Test "+str(name)+ "not found")
        return test

    def get_all_tests(self):
        return self.tests.values()

    def get_executed_tests(self):
        executed_tests = []
        for test in self.tests.values():
            if test.test_result is not None:
                executed_tests.append(test)
        return executed_tests

    def get_passed_tests(self):
        passed_tests = []
        for test in self.tests.values():
            if test.test_result is not None:
                if test.test_result.result:
                    passed_tests.append(test)
        return passed_tests

    def get_failed_tests(self):
        failed_tests = []
        for test in self.tests.values():
            if test.test_result is not None:
                if test.test_result.result == False:
                    failed_tests.append(test)
        return failed_tests

class Test(object):
    def __init__(self, name, description=None, test_result=None, query=None, expected={}):
        self.name = name
        self.description = description
        self.query = query
        self.expected = expected
        self.test_result = test_result

    def to_json_dict(self):
        # if isinstance(self.expected, list):
        #     expected = self._convert_expected(self.expected)
        # elif isinstance(self.expected, dict):
        #     expected = self.expected
        expected = self.expected
        json_dict = {'name': str(self.name), 'description': str(self.description), 'query': str(self.query), 'expected':str(expected)}
        return json_dict

    def _convert_expected(self, expected):
        new_expected = []
        for exp_dict in self.expected:
            new_dict = {}
            for key, value in exp_dict.iteritems():
                new_dict[str(key)] = str(value)
                print "\nkey value of exp_dict"
                print key
                print value
            print "\nexp="
            print str(exp_dict)
            new_expected.append(new_dict)
        print '\n new expected='+str(new_expected)
        return new_expected

    def to_ros_msg(self):
        ros_test = RosTest()
        ros_test.name = str(self.name)
        ros_test.description = str(self.description)
        ros_test.query = str(self.query)
        ros_test.expected = []
        for obj in self.expected:
            for key, value in obj.iteritems():
                ros_expected = RosBinding()
                ros_expected.key = str(key)
                ros_expected.value = str(value)
                ros_test.expected.append(ros_expected)
        if self.test_result is not None:
            ros_test.test_result = self.test_result.to_ros_msg()
        return ros_test

class TestResult(object):
    def __init__(self, result = None, execution_date=None):
        self.result = result
        self.execution_date = execution_date
        self.bindings = {}
        self.notable_time_points = []

    def add_notable_time_point(self, ntp_dict):
        ntp_data = ntp_dict['time']
        ntp = rospy.Time(ntp_data['sec'], ntp_data['nsec'])
        self.notable_time_points.append(ntp)

    def add_notable_time_point_by_values(self, sec, nsec):
        time_point = {'sec': sec, 'nsec': nsec}
        time_point_as_dict = {'time': time_point}
        self.add_notable_time_point(time_point_as_dict)

    def to_ros_msg(self):
        ros_test_result = RosTestResult()
        ros_test_result.result = self.result
        ros_test_result.executionDate = str(self.execution_date)
        ros_test_result.bindings = []
        for obj in self.bindings:
            for key, value in obj.iteritems():
                ros_binding = RosBinding()
                ros_binding.key = str(key)
                ros_binding.value = str(value)
                ros_test_result.bindings.append(ros_binding)
        ros_test_result.notableTimePoints = self.notable_time_points
        return ros_test_result
