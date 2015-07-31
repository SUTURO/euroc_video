package de.suturo.video.robocop.tests;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import net.sf.json.JSONArray;
import net.sf.json.JSONObject;
import de.suturo.video.robocop.query.JSONQuery;
import de.suturo.video.robocop.types.Time;

/**
 * Representation of a single robocop test
 * 
 * @author Moritz Horstmann
 *
 */
public class CopTest {

    private final String name;
    private final String query;
    private static final SimpleDateFormat SDF = new SimpleDateFormat("Y-MM-dd_h:m:s");
    private final List<JSONObject> expected;

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
        Object exp = singleTest.get("expected");
        expected = new ArrayList<>();
        if (exp instanceof JSONObject) {
            expected.add((JSONObject) exp);
        } else if (exp instanceof JSONArray) {
            for (Object o : ((JSONArray) exp)) {
                if (o instanceof JSONObject) {
                    expected.add((JSONObject) o);
                }
            }
        }
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
        test.put("executionDate", SDF.format(new Date()));
        test.put("result", Boolean.valueOf(result));
        test.put("bindings", bindings);
        test.put("notableTimePoints", notableTimes(bindings));
        return test;
    }

    private static JSONArray notableTimes(JSONObject bindings) {
        JSONArray times = new JSONArray();
        if (bindings.containsKey("NTP") && bindings.get("NTP") instanceof JSONArray) {
            JSONArray ntp = (JSONArray) bindings.get("NTP");
            for (Object object : ntp) {
                times.add(Time.getTimepoint((String) object));
            }
            bindings.remove("NTP");
        }
        return times;
    }

    private boolean checkExpected(JSONObject bindings) {
        for (JSONObject bindConfig : expected) {
            for (Object key : bindings.keySet()) {
                if (bindConfig.containsKey(key) && !bindConfig.get(key).equals(bindings.get(key))) {
                    continue;
                }
            }
            for (Object key : bindConfig.keySet()) {
                if (!bindings.containsKey(key)) {
                    continue;
                }
            }
            return true;
        }
        return false;
    }
}
