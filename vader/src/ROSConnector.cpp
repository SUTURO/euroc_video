#include <ROSConnector.h>
#include <exception>
ROSConnector::ROSConnector(void)
{
    n = ros::NodeHandle();
    simulationRunsClient = n.serviceClient<suturo_video_msgs::GetSimulationRuns>("/voldemort/get_simulation_runs");
    testResultsClient = n.serviceClient<suturo_video_msgs::GetTestResults>("/voldemort/get_test_results");
}

std::vector<std::string> ROSConnector::getSimulationRuns()
{
    try
    {
        suturo_video_msgs::GetSimulationRuns srv;
        if(simulationRunsClient.call(srv))
        {
            // TODO: check size and throw exception if size()==0
            std::cout << "srv responded:" << std::endl;
            std::cout << srv.response.database_names.size() << std::endl;
            return srv.response.database_names;
        }
        else
        {
            ROS_ERROR("Failed to call service /voldemort/get_simulation_runs");
            //throw ServiceUnavailableException;
        }
    }
    catch (const std::exception &exc)
    {
        std::cerr << "NOPE NOPE NOPE" << std::endl;
        std::cerr << exc.what();
    }
}

class ServiceUnavailableException
{
};

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "vader_client");
//  if (argc != 3)
//  {
//    ROS_INFO("usage: add_two_ints_client X Y");
//    return 1;
//  }

//  ros::NodeHandle n;
//  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
//  beginner_tutorials::AddTwoInts srv;
//  srv.request.a = atoll(argv[1]);
//  srv.request.b = atoll(argv[2]);
//  if (client.call(srv))
//  {
//    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
//  }
//  else
//  {
//    ROS_ERROR("Failed to call service add_two_ints");
//    return 1;
//  }

//  return 0;
//}
