#include <ROSConnector.h>
#include <exception>

ROSConnector::ROSConnector(void)
{
    n = ros::NodeHandle();
    simulationRunsServiceName = "/voldemort/get_simulation_runs";
    testResultsServiceName = "/voldemort/get_test_results";
    simulationRunsClient = n.serviceClient<suturo_video_msgs::GetSimulationRuns>(simulationRunsServiceName);
    testResultsClient = n.serviceClient<suturo_video_msgs::GetTestResults>(testResultsServiceName);
}

std::vector<std::string> ROSConnector::getSimulationRuns()
{
    suturo_video_msgs::GetSimulationRuns srv;
    if(simulationRunsClient.call(srv))
    {
        return srv.response.database_names;
    }
    else
    {
        std::ostringstream msg;
        msg << "Service unavailable: " << simulationRunsServiceName;
        throw ServiceUnavailableException(msg.str());
    }
}

std::string ROSConnector::getTestResults(std::string run)
{
    suturo_video_msgs::GetTestResults srv;
    srv.request.database_name = run;
    if(testResultsClient.call(srv))
    {
        return srv.response.test_results;
    }
    else
    {
        std::ostringstream msg;
        msg << "Service unavailable: " << simulationRunsServiceName;
        throw ServiceUnavailableException(msg.str());
    }
}
