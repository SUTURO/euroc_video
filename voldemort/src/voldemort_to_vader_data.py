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
        self.runs[run.name] = run

    def get_run_by_name(self, name):
        run = None
        try:
            run = self.runs[name]
        except KeyError:
            print("Run not found, name=" + name)
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
        test.expected = test_data['expected']

        self.tests[name] = test
        return name

    def add_test_result_from_dict(self, test_result_data):
        test_result = TestResult()
        test_name = test_result_data['name']

        test_result.execution_date = test_result_data['executionDate']
        test_result.result = test_result_data['result']
        test_result.bindings = test_result_data['bindings']

        for time_point in test_result_data['notableTimePoints']:
            test_result.add_notable_time_point(time_point)

        test = self.get_test_by_name(test_name)
        test.test_result = test_result

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
        print "Executed Tests: \n"
        for test in executed_tests:
            print test.name, test.test_result.result
        return executed_tests

    def get_passed_tests(self):
        passed_tests = []
        for test in self.tests.values():
            if test.test_result is not None:
                if test.test_result.result:
                    passed_tests.append(test)
        print "Passed Tests: \n"
        print passed_tests
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
        json_dict = {'name': self.name, 'description': self.description, 'query': self.query, 'expected':self.expected}
        return json_dict

    def to_ros_msg(self):
        ros_test = RosTest()
        ros_test.name = str(self.name)
        ros_test.description = str(self.description)
        ros_test.query = str(self.query)
        ros_test.expected = []
        for key, value in self.expected.iteritems():
            ros_expected = RosBinding()
            ros_expected.key = str(key)
            ros_expected.value = str(value)
            ros_test.expected.append(ros_expected)
        if self.test_result is not None:
            ros_test.test_result = self.test_result.to_ros_msg()
            print("TEST to_ros_msg:"+str(ros_test.test_result))
        else:
            print("TEST to_tor_msg NO TEST_RESULT")
        return ros_test

class TestResult(object):
    def __init__(self, result = None, execution_date=None, notable_time_points=[], bindings={}):
        self.result = result
        self.execution_date = execution_date
        self.bindings = bindings
        self.notable_time_points = notable_time_points

    def add_notable_time_point(self, ntp_dict):
        ntp_data = ntp_dict['time']
        ntp = rospy.Time(ntp_data['sec'], ntp_data['nsec'])
        self.notable_time_points.append(ntp)

    def to_ros_msg(self):
        ros_test_result = RosTestResult()
        ros_test_result.result = self.result
        ros_test_result.executionDate = str(self.execution_date)
        ros_test_result.bindings = []
        for key, value in self.bindings.iteritems():
            ros_binding = RosBinding()
            ros_binding.key = str(key)
            ros_binding.value = str(value)
            ros_test_result.bindings.append(ros_binding)
        print "Notable Time Points:"
        print self.notable_time_points
        ros_test_result.notableTimePoints = self.notable_time_points
        return ros_test_result
