package de.suturo.video.robocop.types;

import java.util.Arrays;

import net.sf.json.JSONObject;

/**
 * Represents a time point for vader.
 * 
 * @author Moritz Horstmann
 *
 */
public class Time {

    /**
     * Construct a new time point from the given time point
     * 
     * @param timepoint
     *            a time point from knowrob (eg. 'timepoint_94.123')
     */
    public static JSONObject getTimepoint(String timepoint) {
        String point = timepoint.substring(timepoint.indexOf("_") + 1);
        String[] splitted = point.split("\\.");
        JSONObject outerTime = new JSONObject();
        JSONObject json = new JSONObject();
        json.put("sec", Integer.valueOf(splitted[0]));
        if (splitted[1].length() > 9) {
            splitted[1] = splitted[1].substring(0, 9);
        }
        if (splitted[1].length() < 9) {
            char[] zeroes = new char[9 - splitted[1].length()];
            Arrays.fill(zeroes, '0');
            splitted[1] = splitted[1] + new String(zeroes);
        }
        json.put("nsec", Integer.valueOf(splitted[1]));
        outerTime.put("time", json);
        return outerTime;
    }
}
