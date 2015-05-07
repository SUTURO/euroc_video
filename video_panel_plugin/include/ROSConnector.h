#ifndef ROSCONNECTOR_H
#define ROSCONNECTOR_H

#include "ros/ros.h"
#include "suturo_video_msgs/GetSimulationRuns.h"
#include "suturo_video_msgs/GetTestResults.h"

class ROSConnector
{
public:
    ROSConnector();
    std::vector<std::string> getSimulationRuns();

private:
    ros::NodeHandle n;
    ros::ServiceClient simulationRunsClient;
    ros::ServiceClient testResultsClient;
};

#endif // ROSCONNECTOR_H
