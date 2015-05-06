/*
 * robot_description_mongo_logger_node.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: andz
 */

#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <chrono>
#include <string>
#include <thread>

using namespace mongo;
using std::chrono::duration;
using std::chrono::system_clock;

typedef duration<int> seconds;

DBClientConnection *mongodb_conn;
std::string collection;

int main(int argc, char **argv) {
	std::string mongodb = "localhost";
	collection = "roslog.robot_description";
	seconds timeout(0);

	std::string parametersString = "/robot_description,/robot_description_semantic";
	std::vector<std::string> parameters;

	int c;
	while ((c = getopt(argc, argv, "p:m:t:c:")) != -1) {
		switch (c) {
		case 'p':
			parametersString = optarg;
			break;
		case 'm':
			 mongodb = optarg;
			 break;
		case 'c':
			collection = optarg;
			 break;
		case 't':
			timeout = seconds(std::stoi(optarg));
			break;
		default:
			printf("Usage: %s [-p parameters] [-m mongob] [-c collection] [-t timeout in seconds (int)]\n", argv[0]);
			exit(-1);
		}
	}

	string parameter;
	stringstream stream(parametersString);
	while (getline(stream, parameter, ',')) {
		parameters.push_back(parameter);
	}

	ros::init(argc, argv, "robot_description_mongo_logger");
	ros::NodeHandle nh;

	std::string errmsg;
	mongodb_conn = new DBClientConnection(true);
	if (!mongodb_conn->connect(mongodb, errmsg)) {
		ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
		return -1;
	}

	BSONObjBuilder document;

	std::vector<bool> gotParameter(parameters.size(), false);

	system_clock::time_point beginning = system_clock::now();

	document.append("epoch", beginning.time_since_epoch().count());

	for (size_t received = 0; received < parameters.size() && (timeout == seconds::zero() || system_clock::now() - beginning < timeout);) {
		for (size_t i=0; i<parameters.size(); ++i) {
			std::string &param = parameters[i];
			std::string val;
			bool gotParam = gotParameter[i];
			if(!gotParam && !nh.getParam(param, val)) {
				ROS_WARN_STREAM("Could not get data from " << param << ".");
			} else if (!gotParam) {
				gotParameter[i] = true;
				++received;
				ROS_INFO_STREAM("Got data from " << param << ".");
				document.append(param, val);
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	mongodb_conn->insert(collection, document.obj());
	delete mongodb_conn;

	for (size_t i=0; i<parameters.size(); ++i) {
		if(!gotParameter[i]) {
			ROS_ERROR_STREAM("Failed to get data from " << parameters[i] << ".");
		}
	}

	return 0;
}
