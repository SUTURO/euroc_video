package de.suturo.video.robocop;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URI;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.ws.rs.GET;
import javax.ws.rs.POST;
import javax.ws.rs.Path;
import javax.ws.rs.Produces;
import javax.ws.rs.QueryParam;
import javax.ws.rs.core.MediaType;

import net.sf.json.JSONArray;

import org.glassfish.grizzly.http.server.HttpServer;
import org.glassfish.jersey.grizzly2.httpserver.GrizzlyHttpServerFactory;
import org.glassfish.jersey.server.ResourceConfig;

import de.suturo.video.robocop.query.JSONQuery;
import de.suturo.video.robocop.query.ThreadedQuery;
import de.suturo.video.robocop.solutions.PrologAllSolutions;
import de.suturo.video.robocop.solutions.PrologSolutions;

/**
 * Robocop server based on json_prolog by Lorenz Moesenlechner and Moritz Tenorth
 *
 * @author Moritz Horstmann
 */

@Path("/robocop")
public class RobocopServer {

    private static final String BASE_URL = "http://localhost:8080";

    /**
     * Executes the given tests
     */
    @GET
    @Path("/executeTest")
    @Produces(MediaType.APPLICATION_JSON)
    public String executeTest() {
        try (ThreadedQuery query = new ThreadedQuery("expand_goal((member(A, [a,b,c])),_Q), call(_Q)")) {
            Thread t = new Thread(query);
            t.start();
            while (!query.isStarted()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    // Don't interrupt my sleep
                }
            }
            PrologSolutions s = new PrologAllSolutions(query);
            JSONArray result = new JSONArray();
            while (s.hasMoreSolutions()) {
                result.add(JSONQuery.encodeResult(s.nextSolution()));
            }
            return result.toString();
        } catch (Exception e) {
            return "error: " + e;
        }
    }

    /**
     * Executes the given tests
     */
    @POST
    @Path("/setOWL")
    @Produces(MediaType.APPLICATION_JSON)
    public String setOWL(@QueryParam("owl") String owl) {
        new jpl.Query("load_experiment('" + owl + "')").oneSolution();
        return "ok";
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
     *
     * @throws IOException
     * @throws InterruptedException
     * @throws RospackError
     */
    private static void initProlog() throws IOException, InterruptedException, RospackError {
        List<String> pl_args = new ArrayList<>(Arrays.asList(jpl.JPL.getDefaultInitArgs()));
        pl_args.set(0, "/usr/bin/swipl");
        pl_args.add("-G256M");
        pl_args.add("-nosignals");
        jpl.JPL.setDefaultInitArgs(pl_args.toArray(new String[0]));
        jpl.JPL.init();
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
