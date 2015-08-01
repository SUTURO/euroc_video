package de.suturo.video.robocop;

import static org.hamcrest.CoreMatchers.is;
import static org.junit.Assert.assertThat;
import net.sf.json.JSONArray;
import net.sf.json.JSONObject;

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
     * Tests if the notable time points are being parsed correctly and are appended to the test result.
     */
    @SuppressWarnings("boxing")
    @Test
    public void ntpTest() {
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Dummy execution\","
                + "        \"query\": \"NTP=['timepoint_96.405', 'timepoint_97.60107', 'timepoint_98.0', 'timepoint_99.12345678912'].\","
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
        assertThat("timepoint 3 seconds", ((JSONObject) points.getJSONObject(2).get("time")).get("sec"), is(98));
        assertThat("timepoint 3 nanos", ((JSONObject) points.getJSONObject(2).get("time")).get("nsec"), is(0));
        assertThat("timepoint 4 seconds", ((JSONObject) points.getJSONObject(3).get("time")).get("sec"), is(99));
        assertThat("timepoint 4 nanos", ((JSONObject) points.getJSONObject(3).get("time")).get("nsec"), is(123456789));
    }
}
