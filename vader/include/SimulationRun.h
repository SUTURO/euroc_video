#ifndef SIMULATIONRUN_H
#define SIMULATIONRUN_H

#include <string>
#include <suturo_video_msgs/Test.h>
struct SimulationRun {
    std::string name;
    std::vector<suturo_video_msgs::Test> availableTests, executedTests, failedTests, passedTests;
};

#endif
