package de.suturo.video.robocop;

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.is;
import static org.junit.Assert.assertThat;
import net.sf.json.JSONArray;
import net.sf.json.JSONObject;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Tests for the execution of robocop tests.<br>
 * This needs to to be run with a configured jpl prolog environment (swi-prolog-java +
 * -Djava.library.path=/usr/lib/swi-prolog/lib/amd64)
 * 
 * @author Moritz Horstmann
 *
 */
public class RobocopTestTests {

    private static RobocopServer instance;

    /**
     * initialize robocop instance
     */
    @BeforeClass
    public static void setUpStatic() {
        instance = new RobocopServer();
        RobocopServer.initProlog();
    }

    /**
     * Fake notable timepoints for tests.
     */
    @Before
    public void setUp() {
        jpl.Query.oneSolution("retractall(getNotableTimepoints(_)).");
        jpl.Query.oneSolution("retractall(initializeNotableTimepoints).");
        jpl.Query.oneSolution("assert(initializeNotableTimepoints :- true).");
        jpl.Query.oneSolution("assert(getNotableTimepoints(NTP) :- NTP = []).");
    }

    /**
     * Tests if a simple robocop test suite can be parsed and executed.
     */
    @SuppressWarnings("boxing")
    @Test
    public void simpleTestSuite() {
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(A, [b]).\"," //
                + "        \"expected\": {" //
                + "                \"A\": \"b\"" //
                + "        }" //
                + "}," //
                + "{" //
                + "        \"name\": \"TESTNAME2\","
                + "        \"description\": \"Yet another dummy execution, should fail\","
                + "        \"query\": \"member(A, [nota]).\"," //
                + "        \"expected\": {" //
                + "                \"A\": \"a\"" //
                + "        }" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
        assertThat("result of test execution", ((JSONObject) result.get(1)).getBoolean("result"), is(false));
    }

    /**
     * Tests if a simple robocop test suite with new structure can be parsed.
     */
    @Test
    public void simpleExtendedTestSuite() {
        String test = "{"
                + "\"name\": \"Testsuite\"," //
                + "\"dependencies\": [\"firstDep\", \"secondDep\"]," //
                + "\"tests\": [" //
                + "     {" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(A, [b]).\"," //
                + "        \"expected\": {" //
                + "                \"A\": \"b\"" //
                + "        }" //
                + "     }," //
                + "     {" //
                + "        \"name\": \"TESTNAME2\","
                + "        \"description\": \"Yet another dummy execution, should fail\","
                + "        \"query\": \"member(A, [nota]).\"," //
                + "        \"expected\": {" //
                + "                \"A\": \"a\"" //
                + "        }" //
                + "     }" //
                + "     ]" //
                + "}";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
    }

    /**
     * Tests if a list of multiple expected bindings can be tested. The following cases are tested here:
     * <ol>
     * <li>One expected binding is tested for a query which returns multiple answers. The expected binding is amongst
     * the returned answers, so the result should be true.
     * <li>Multiple expected bindings are tested for a query which returns multiple answers. The expected bindings are
     * amongst the returned answers, so the result should be true.
     * <li>Multiple expected bindings are tested for a query which returns multiple answers. The expected bindings are
     * not amongst the returned answers, so the result should be false.
     * </ol>
     */
    @SuppressWarnings("boxing")
    @Test
    public void multiResultTestSuite() {
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(A, [b, c, d]).\"," //
                + "        \"expected\": [" //
                + "             { \"A\": \"d\" }" //
                + "        ]" //
                + "}," //
                + "{" //
                + "        \"name\": \"TESTNAME2\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(A, [b, c, d]).\"," //
                + "        \"expected\": [" //
                + "             { \"A\": \"d\" }," //
                + "             { \"A\": \"c\" }" //
                + "        ]" //
                + "}," //
                + "{" //
                + "        \"name\": \"TESTNAME3\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(A, [b, c, d]).\"," //
                + "        \"expected\": [" //
                + "             { \"A\": \"x\" }," //
                + "             { \"A\": \"y\" }" //
                + "        ]" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
        assertThat("result of test execution", ((JSONObject) result.get(1)).getBoolean("result"), is(true));
        assertThat("result of test execution", ((JSONObject) result.get(2)).getBoolean("result"), is(false));
    }

    /**
     * Tests if an unexpected test execution error is handled correctly and does not affect the following tests.
     */
    @SuppressWarnings("boxing")
    @Test
    public void testWithError() {
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"jibbetnich.\"," //
                + "        \"expected\": [" //
                + "             { \"A\": \"d\" }" //
                + "        ]" //
                + "}," //
                + "{" //
                + "        \"name\": \"TESTNAME2\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(A, [b, c, d]).\"," //
                + "        \"expected\": [" //
                + "             { \"A\": \"d\" }," //
                + "             { \"A\": \"c\" }" //
                + "        ]" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getString("result"), is("error"));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getString("error"),
                containsString("existence_error"));
        assertThat("result of test execution", ((JSONObject) result.get(1)).getBoolean("result"), is(true));
    }

    /**
     * Tests if multiple bindings of the same value don't trick the test in to passing.
     */
    @SuppressWarnings("boxing")
    @Test
    public void negativeWithDuplicate() {
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"member(Object, [green_cylinder, blue_handle, green_cylinder, blue_handle]).\"," //
                + "        \"expected\": [" //
                + "             { \"Object\": \"blue_handle\" }," //
                + "             { \"Object\": \"green_cylinder\" }," //
                + "             { \"Object\": \"red_cube\" }" //
                + "        ]" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(false));
        assertThat("first binding",
                ((JSONObject) result.get(0)).getJSONArray("bindings").getJSONObject(0).getString("Object"),
                is("green_cylinder"));
    }

    /**
     * Tests if the notable time points are being parsed correctly and are appended to the test result.
     */
    @SuppressWarnings("boxing")
    @Test
    public void ntpTest() {
        jpl.Query
                .oneSolution("retractall(getNotableTimepoints(_)), assertz(getNotableTimepoints(NTP) :- NTP = ['timepoint_96.405', 'timepoint_97.60107', 'http://knowrob.org/kb/cram_log.owl#timepoint_116.208', 'timepoint_99.12345678912']).");
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"getNotableTimepoints\": true,"
                + "        \"query\": \"member(A, [c]).\","
                + "        \"expected\": {" //
                + "                \"A\": \"b\"" //
                + "        }" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(false));
        JSONArray points = (JSONArray) ((JSONObject) result.get(0)).get("notableTimePoints");
        assertThat("timepoint 1 seconds", ((JSONObject) points.getJSONObject(0).get("time")).get("sec"), is(96));
        assertThat("timepoint 1 nanos", ((JSONObject) points.getJSONObject(0).get("time")).get("nsec"), is(405000000));
        assertThat("timepoint 2 seconds", ((JSONObject) points.getJSONObject(1).get("time")).get("sec"), is(97));
        assertThat("timepoint 2 nanos", ((JSONObject) points.getJSONObject(1).get("time")).get("nsec"), is(601070000));
        assertThat("timepoint 3 seconds", ((JSONObject) points.getJSONObject(2).get("time")).get("sec"), is(116));
        assertThat("timepoint 3 nanos", ((JSONObject) points.getJSONObject(2).get("time")).get("nsec"), is(208000000));
        assertThat("timepoint 4 seconds", ((JSONObject) points.getJSONObject(3).get("time")).get("sec"), is(99));
        assertThat("timepoint 4 nanos", ((JSONObject) points.getJSONObject(3).get("time")).get("nsec"), is(123456789));
    }

    /**
     * Tests if line breaks inside the JSON can distort a test.
     */
    @SuppressWarnings("boxing")
    @Test
    public void lineBreakCompatibility() {
        String test = "[\n" //
                + "{\n" //
                + "        \"name\": \"TESTNAME1\",\n"
                + "        \"description\": \"Dummy execution\",\n"
                + "        \"query\": \"member(Object, [green_cylinder, blue_handle, green_cylinder, blue_handle]).\",\n" //
                + "        \"expected\": [\n" //
                + "             { \"Object\": \"blue_handle\" },\n" //
                + "             { \"Object\": \"green_cylinder\" },\n" //
                + "             { \"Object\": \"red_cube\" }\n" //
                + "        ]\n" //
                + "}\n" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(false));
        assertThat("first binding",
                ((JSONObject) result.get(0)).getJSONArray("bindings").getJSONObject(0).getString("Object"),
                is("green_cylinder"));
    }

    /**
     * Tests if python unicode strings are ignored properly.
     */
    @SuppressWarnings("boxing")
    @Test
    public void ignorePythonStrings() {
        String test = "[" //
                + "{" //
                + "        u'name': u'TESTNAME1',"
                + "        u'description': u'Dummy execution',"
                + "        u'query': u'member(Object, [green_cylinder, blue_handle, green_cylinder, blue_handle]).'," //
                + "        u'expected': [" //
                + "             { u'Object': u'blue_handle' }," //
                + "             { u'Object': u'green_cylinder' }" //
                + "        ]" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
        assertThat("first binding",
                ((JSONObject) result.get(0)).getJSONArray("bindings").getJSONObject(0).getString("Object"),
                is("green_cylinder"));
    }

    /**
     * Tests if a simple test which returns true, can be executed and asserted in robocop.
     */
    @SuppressWarnings("boxing")
    @Test
    public void testNoBindingSuccess() {
        String test = "[" //
                + "{" //
                + "        'name': 'TESTNAME1',"
                + "        'description': 'Dummy execution',"
                + "        'query': '4 is 4.'," //
                + "        'expected': [" //
                + "             {}" //
                + "        ]" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
    }

    /**
     * Tests if a simple test which returns false, can be executed and asserted in robocop.
     */
    @SuppressWarnings("boxing")
    @Test
    public void testNoBindingFail() {
        String test = "[" //
                + "{" //
                + "        'name': 'TESTNAME1',"
                + "        'description': 'Dummy execution',"
                + "        'query': '4 is 4.'," //
                + "        'expected': [" //
                + "             {}" //
                + "        ]" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
    }

    /**
     * Tests if a simple test which returns false, can be executed and asserted in robocop.
     */
    @SuppressWarnings("boxing")
    @Test
    public void testOnlyFirstSuccess() {
        String test = "[" //
                + "{" //
                + "        'name': 'TESTNAME1',"
                + "        'description': 'Dummy execution',"
                + "        'query': '4 is 4; 4 is 3.'," //
                + "        'expected': {}" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
    }
}
