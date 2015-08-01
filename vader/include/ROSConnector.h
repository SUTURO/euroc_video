#ifndef ROSCONNECTOR_H
#define ROSCONNECTOR_H

#include "ros/ros.h"
#include "suturo_video_msgs/GetSimulationRuns.h"
#include "suturo_video_msgs/GetTests.h"
#include "suturo_video_msgs/Test.h"
#include "suturo_video_msgs/TestResult.h"
#include "suturo_video_msgs/GetTopicNames.h"
#include "suturo_video_msgs/AddTests.h"
#include "suturo_video_msgs/ExecuteTests.h"

class ROSConnector
{
public:
    ROSConnector();
    std::vector<std::string> getSimulationRuns();
    std::vector<suturo_video_msgs::Test> getAvailableTests(std::string run);
    std::vector<suturo_video_msgs::Test> getExecutedTests(std::string run);
    std::vector<suturo_video_msgs::Test> getFailedTests(std::string run);
    std::vector<suturo_video_msgs::Test> getPassedTests(std::string run);
    std::vector<std::string> getPlayableTopics(std::string run);
    bool addTests(std::string filePath);
    bool executeTests(std::string run);

private:
    ros::NodeHandle n;
    ros::ServiceClient simulationRunsClient;
    ros::ServiceClient availableTestsClient;
    ros::ServiceClient executedTestsClient;
    ros::ServiceClient failedTestsClient;
    ros::ServiceClient passedTestsClient;
    ros::ServiceClient getPlayableTopicsClient;
    ros::ServiceClient addTestsClient;
    ros::ServiceClient executeTestsClient;
    std::string simulationRunsServiceName;
    std::string availableTestsServiceName;
    std::string executedTestsServiceName;
    std::string failedTestsServiceName;
    std::string passedTestsServiceName;
    std::string getPlayableTopicsServiceName;
    std::string addTestsServiceName;
    std::string executeTestsServiceName;
};

#endif // ROSCONNECTOR_H

class ServiceUnavailableException : public std::runtime_error
{
public: ServiceUnavailableException(std::string exc): runtime_error(exc) {}
};
