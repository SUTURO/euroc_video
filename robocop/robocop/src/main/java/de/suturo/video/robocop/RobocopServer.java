package de.suturo.video.robocop;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URI;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.ws.rs.FormParam;
import javax.ws.rs.GET;
import javax.ws.rs.PUT;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;
import javax.ws.rs.core.MediaType;

import net.sf.json.JSONObject;

import org.glassfish.grizzly.http.server.HttpServer;
import org.glassfish.jersey.grizzly2.httpserver.GrizzlyHttpServerFactory;
import org.glassfish.jersey.server.ResourceConfig;

import de.suturo.video.robocop.tests.CopTestSuite;
import de.suturo.video.robocop.tests.ParseException;

/**
 * Robocop server based on json_prolog by Lorenz Moesenlechner and Moritz Tenorth
 *
 * @author Moritz Horstmann
 */

@Path("/robocop")
public class RobocopServer {
    /** default ok result */
    static final String RESULT_OK = "{result: \"ok\"}";
    private static final String BASE_URL = "http://localhost:8080";

    /**
     * Executes the given tests
     */
    @GET
    @Path("/executeTest")
    @Produces(MediaType.APPLICATION_JSON)
    public String executeTest(@QueryParam("owl") String owl, @QueryParam("db") String db) {
        if (owl != null && !"".equals(owl)) {
            // new jpl.Query("load_experiment('" + owl + "')").oneSolution();
        }
        if (db != null && !"".equals(db)) {
            // new jpl.Query("mang_db('" + db + "')").oneSolution();
        }
        try {
            return new StubTest().toString();// testSuite.executeSuite().toString();
        } catch (Exception e) {
            return jsonError(e);
        }
    }

    /**
     * Upload method for robocop tests.
     */
    @PUT
    @Path("/uploadTest")
    @Produces(MediaType.APPLICATION_JSON)
    public String uploadTest(@FormParam("test") String test) {
        try {
            /** this.testSuite = */
            new CopTestSuite(test);
            return RESULT_OK;
        } catch (ParseException e) {
            return jsonError(e);
        }
    }

    private static String jsonError(Exception e) {
        JSONObject err = new JSONObject();
        err.put("result", "error");
        err.put("error", e.getMessage());
        return err.toString();
    }

    /**
     * Main routine handling all the REST server stuff for robocop.
     * 
     * @param args
     */
    public static void main(String... args) throws Exception {
        final ResourceConfig rc = new ResourceConfig().packages("de.suturo.video.robocop");
        final HttpServer copserver = GrizzlyHttpServerFactory.createHttpServer(URI.create(BASE_URL), rc);
        initProlog();
        if (!Arrays.asList(args).contains("--no-ros")) {
            initROS();
        }
        while (true) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                break;
            }
        }
        copserver.shutdown();
    }

    /**
     * Initialize the SWI Prolog engine
     */
    static void initProlog() {
        List<String> pl_args = new ArrayList<>(Arrays.asList(jpl.JPL.getDefaultInitArgs()));
        pl_args.set(0, "/usr/bin/swipl");
        pl_args.add("-G256M");
        pl_args.add("-nosignals");
        jpl.JPL.setDefaultInitArgs(pl_args.toArray(new String[0]));
        jpl.JPL.init();
    }

    /**
     * Initialize rosprolog
     *
     */
    static void initROS() throws IOException, InterruptedException, RospackError {
        new jpl.Query("ensure_loaded('" + findRosPackage("rosprolog") + "/prolog/init.pl')," //
                + "ensure_loaded('" + findRosPackage("robocop") + "/prolog/init.pl')").oneSolution();
    }

    /**
     * Find a ROS package using the rospack program
     *
     * @param name
     *            Name of the ROS package
     * @return Path to the ROS package
     * @throws IOException
     * @throws InterruptedException
     * @throws RospackError
     */
    private static String findRosPackage(String name) throws IOException, InterruptedException, RospackError {
        Process rospack = Runtime.getRuntime().exec("rospack find " + name);
        if (rospack.waitFor() != 0)
            throw new RospackError();
        return new BufferedReader(new InputStreamReader(rospack.getInputStream())).readLine();
    }

    static private class RospackError extends Exception {
        public RospackError() {
            // default constructor
        }

        private static final long serialVersionUID = 1L;
    }

}
