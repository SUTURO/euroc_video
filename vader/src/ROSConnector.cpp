#include <ROSConnector.h>
#include <exception>

ROSConnector::ROSConnector(void)
{
    n = ros::NodeHandle();
    simulationRunsServiceName = "/voldemort/get_simulation_runs";
    testResultsServiceName = "/voldemort/get_test_results";
    simulationRunsClient = n.serviceClient<suturo_video_msgs::GetSimulationRuns>(simulationRunsServiceName);
    testResultsClient = n.serviceClient<suturo_video_msgs::GetTests>(testResultsServiceName);
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

std::string ROSConnector::getTestResults(std::string run)
{
    suturo_video_msgs::GetTests srv;
    srv.request.simulation_run_name = run;
    if(testResultsClient.call(srv))
    {
        return "success";
    }
    else
    {
        std::ostringstream msg;
        msg << "Service unavailable: " << simulationRunsServiceName;
        throw ServiceUnavailableException(msg.str());
    }
}
