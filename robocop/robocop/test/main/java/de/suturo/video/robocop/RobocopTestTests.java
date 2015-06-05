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
     * tests if a simple robocop test suite can be parsed and executed.
     */
    @SuppressWarnings("boxing")
    @Test
    public void simpleTestSuite() {
        String test = "[" //
                + "{" //
                + "        \"name\": \"TESTNAME1\","
                + "        \"description\": \"Tests if three objects were placed\","
                + "        \"query\": \"member(A, [b]).\"," //
                + "        \"expected\": {" //
                + "                \"A\": \"b\"" //
                + "        }" //
                + "}" //
                + "]";
        assertThat("result of test upload", instance.uploadTest(test), is(RobocopServer.RESULT_OK));
        JSONArray result = JSONArray.fromObject(instance.executeTest(null, null));
        assertThat("result of test execution", ((JSONObject) result.get(0)).getBoolean("result"), is(true));
    }
}
