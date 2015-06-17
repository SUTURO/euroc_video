#ifndef ROSCONNECTOR_H
#define ROSCONNECTOR_H

#include "ros/ros.h"
#include "suturo_video_msgs/GetSimulationRuns.h"
#include "suturo_video_msgs/GetTests.h"

class ROSConnector
{
public:
    ROSConnector();
    std::vector<std::string> getSimulationRuns();
    std::string getTestResults(std::string run);

private:
    ros::NodeHandle n;
    ros::ServiceClient simulationRunsClient;
    ros::ServiceClient testResultsClient;
    std::string simulationRunsServiceName;
    std::string testResultsServiceName;
};

#endif // ROSCONNECTOR_H

class ServiceUnavailableException : public std::runtime_error
{
public: ServiceUnavailableException(std::string exc): runtime_error(exc) {}
};
