class SimulationRun(object):
    def __init__(self, name, tests={}):
        self.name = name
        self.tests = tests

    def add_test(self, test):
        self.tests[test.name] = test

    def get_test_by_name(self, name):
        test = None
        try:
            test = self.tests[name]
        except KeyError:
            print("Test "+name+ "not found for Run"+self.name)
        return test


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
        self.tests[test.name] = test

    def get_test_by_name(self, name):
        test = None
        try:
            test = self.tests[name]
        except KeyError:
            print("Test "+name+ "not found")
        return test


class Test(object):
    def __init__(self, name, description=None, test_result=None, query=None, expected=None):
        self.name = name
        self.description = description
        self.query = query
        self.expected = expected

        self.test_result = test_result

    def to_json_dict(self):
        json_dict = {'name': self.name, 'description': self.description, 'query': self.query, 'expected':self.expected}
        return json_dict


class TestResult(object):
    def __init__(self, result = None, execution_date=None):
        self.result = result
        self.execution_date = execution_date