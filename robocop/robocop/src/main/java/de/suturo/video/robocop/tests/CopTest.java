package de.suturo.video.robocop.tests;

import java.text.SimpleDateFormat;
import java.util.Date;

import net.sf.json.JSONObject;
import de.suturo.video.robocop.query.JSONQuery;

/**
 * Representation of a single robocop test
 * 
 * @author Moritz Horstmann
 *
 */
public class CopTest {

    private final String name;
    private final String query;
    private static final SimpleDateFormat sdf = new SimpleDateFormat("Y-MM-dd_h:m:s");
    private final JSONObject expected;

    /**
     * Creates a new robocop test from the given JSON object.
     * 
     * @param singleTest
     *            JSON object representing a single robocop test
     */
    public CopTest(JSONObject singleTest) {
        name = singleTest.getString("name");
        // singleTest.getString("description"); not used atm
        query = singleTest.getString("query");
        expected = singleTest.getJSONObject("expected");
    }

    /**
     * * Executes the test represented by this object. This requires a configured and set up swi-prolog jni environment!
     */
    JSONObject execute() {
        @SuppressWarnings("unchecked")
        JSONObject bindings = JSONQuery.encodeResult(jpl.Query.oneSolution(query));
        boolean result = checkExpected(bindings);
        JSONObject test = new JSONObject();
        test.put("name", name);
        test.put("executionDate", sdf.format(new Date()));
        test.put("result", Boolean.valueOf(result));
        test.put("bindings", bindings);
        return test;
    }

    private boolean checkExpected(JSONObject bindings) {
        for (Object key : bindings.keySet()) {
            if (!expected.containsKey(key) || !expected.get(key).equals(bindings.get(key))) {
                return false;
            }
        }
        return true;
    }
}
