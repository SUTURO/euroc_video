package de.suturo.video.robocop;

import java.text.SimpleDateFormat;
import java.util.Date;

public class StubTest {
    private static final SimpleDateFormat sdf = new SimpleDateFormat("Y-MM-dd_h:m:s");

    @Override
    public String toString() {
        return "[{\"name\": \"Three-Objects\",\"executionDate\": \""
                + sdf.format(new Date())
                + "\",\"result\": false,\"bindings\": {\"ObjectsFound\": [\"blue_handle\", \"green_cylinder\"]},\"notableTimePoints\":[{\"time\": {\"sec\": 188,\"nsec\": 9000000}},{\"time\": {\"sec\": 99,\"nsec\": 937000000}}]}]";
    }
}
