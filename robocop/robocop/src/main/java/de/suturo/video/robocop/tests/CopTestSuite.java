package de.suturo.video.robocop.tests;

import java.util.ArrayList;
import java.util.List;

import net.sf.json.JSONArray;
import net.sf.json.JSONObject;

/**
 * Representation of a complete robocop test suite
 * 
 * @author Moritz Horstmann
 *
 */
public class CopTestSuite {

    private final List<CopTest> tests = new ArrayList<>();

    private final String name;

    private final List<String> dependencies = new ArrayList<>();

    /**
     * Constructs a robocop test suite from the given JSON string.
     * 
     * @param json
     *            JSON array of valid robocop tests.
     * @throws ParseException
     */
    public CopTestSuite(String json) throws ParseException {
        String sanitized = filterPython(json.trim());
        JSONArray testsObject;
        if (json.startsWith("{")) {
            JSONObject suite = JSONObject.fromObject(sanitized);
            name = suite.getString("name");
            for (Object dep : suite.getJSONArray("dependencies")) {
                if (!(dep instanceof String)) {
                    throw new ParseException("Invalid dependency value " + dep);
                }
                dependencies.add((String) dep);
            }
            testsObject = suite.getJSONArray("tests");
        } else {
            name = null;
            testsObject = JSONArray.fromObject(sanitized);
        }
        for (Object singleTest : testsObject) {
            if (!(singleTest instanceof JSONObject)) {
                throw new ParseException("Expected a JSON key/value object, got: " + singleTest);
            }
            tests.add(new CopTest((JSONObject) singleTest));
        }
    }

    /**
     * Executes all tests inside this suite and returns a JSON array with the results.
     */
    public JSONArray executeSuite() {
        JSONArray results = new JSONArray();
        for (CopTest test : tests) {
            try {
                results.add(test.execute());
            } catch (Exception e) {
                results.add(jsonError(test, e));
            }
        }
        return results;
    }

    private static JSONObject jsonError(CopTest test, Exception e) {
        JSONObject err = new JSONObject();
        err.put("name", test.getName());
        err.put("result", "error");
        err.put("error", e.getMessage());
        return err;
    }

    private static String filterPython(String input) {
        return input.replace("u'", "'");
    }

    /**
     * @return the name
     */
    public String getName() {
        return name;
    }

    /**
     * @return the dependencies
     */
    public List<String> getDependencies() {
        return dependencies;
    }
}
