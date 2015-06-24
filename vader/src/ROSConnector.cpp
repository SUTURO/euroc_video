#include <ROSConnector.h>
#include <exception>


ROSConnector::ROSConnector(void)
{
    n = ros::NodeHandle();

    // declare service names
    simulationRunsServiceName = "/voldemort/get_simulation_runs";
    availableTestsServiceName = "/voldemort/get_available_tests";
    executedTestsServiceName = "/voldemort/get_executed_tests";
    failedTestsServiceName = "/voldemort/get_failed_tests";
    passedTestsServiceName = "/voldemort/get_passed_tests";

    // create service clients
    simulationRunsClient = n.serviceClient<suturo_video_msgs::GetSimulationRuns>(simulationRunsServiceName);
    availableTestsClient = n.serviceClient<suturo_video_msgs::GetTests>(availableTestsServiceName);
    executedTestsClient = n.serviceClient<suturo_video_msgs::GetTests>(executedTestsServiceName);
    failedTestsClient = n.serviceClient<suturo_video_msgs::GetTests>(failedTestsServiceName);
    passedTestsClient = n.serviceClient<suturo_video_msgs::GetTests>(passedTestsServiceName);
}

std::vector<std::string> ROSConnector::getSimulationRuns()
{
    suturo_video_msgs::GetSimulationRuns srv;
    if(simulationRunsClient.call(srv))
    {
        return srv.response.simulation_runs;
    }
    else
    {
        std::ostringstream msg;
        msg << "Service unavailable: " << simulationRunsServiceName;
        throw ServiceUnavailableException(msg.str());
    }
}

std::vector<suturo_video_msgs::Test> ROSConnector::getAvailableTests(std::string run)
{
    suturo_video_msgs::GetTests srv;
    srv.request.simulation_run_name = run;
    if(availableTestsClient.call(srv))
    {
        return srv.response.tests;
    }
    else
    {
        std::ostringstream msg;
        msg << "Service unavailable: " << availableTestsServiceName;
        throw ServiceUnavailableException(msg.str());
    }
}

std::vector<suturo_video_msgs::Test> ROSConnector::getExecutedTests(std::string run)
{
    suturo_video_msgs::GetTests srv;
    srv.request.simulation_run_name = run;
    if(executedTestsClient.call(srv))
    {
        return srv.response.tests;
    }
    else
    {
        std::ostringstream msg;
        msg << "Service unavailable: " << executedTestsServiceName;
        throw ServiceUnavailableException(msg.str());
    }
}
