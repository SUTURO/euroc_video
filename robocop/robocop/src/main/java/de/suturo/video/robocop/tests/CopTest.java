package de.suturo.video.robocop.tests;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Hashtable;
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
    private static final SimpleDateFormat SDF = new SimpleDateFormat("YYYY-MM-dd_HH:mm:ss");
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
        List<JSONObject> bindings = allAnswers();
        boolean result = checkMultiExpected(bindings);
        JSONObject test = new JSONObject();
        test.put("name", name);
        test.put("executionDate", SDF.format(new Date()));
        test.put("result", Boolean.valueOf(result));
        test.put("bindings", bindings);
        test.put("notableTimePoints", notableTimes(bindings));
        return test;
    }

    @SuppressWarnings({ "unchecked", "rawtypes" })
    private List<JSONObject> allAnswers() {
        List<JSONObject> bindings = new ArrayList<>();
        for (Hashtable binding : jpl.Query.allSolutions(query)) {
            bindings.add(JSONQuery.encodeResult(binding));
        }
        return bindings;
    }

    private static JSONArray notableTimes(List<JSONObject> bindings) {
        JSONArray times = new JSONArray();
        for (JSONObject binding : bindings) {
            if (binding.containsKey("NTP") && binding.get("NTP") instanceof JSONArray) {
                JSONArray ntp = (JSONArray) binding.get("NTP");
                for (Object object : ntp) {
                    times.add(Time.getTimepoint((String) object));
                }
                binding.remove("NTP");
            }
        }
        return times;
    }

    /**
     * Returns true if all expected bindings exist exactly in the list of given bindings in any order.
     */
    private boolean checkMultiExpected(List<JSONObject> bindings) {
        List<JSONObject> remainingTests = new ArrayList<>(expected);
        for (JSONObject test : remainingTests) {
            if (!checkExpected(test, bindings)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true if the given expected bindings match any of the given bindings.
     */
    private static boolean checkExpected(JSONObject expectedBindings, List<JSONObject> allBindings) {
        nextBinding: for (JSONObject bindConfig : allBindings) {
            for (Object key : expectedBindings.keySet()) {
                if (bindConfig.containsKey(key) && !bindConfig.get(key).equals(expectedBindings.get(key))) {
                    continue nextBinding;
                }
            }
            for (Object key : bindConfig.keySet()) {
                if (!expectedBindings.containsKey(key)) {
                    continue nextBinding;
                }
            }
            return true;
        }
        return false;
    }
}
