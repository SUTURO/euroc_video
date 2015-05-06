/*
 * call_robocop.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: andz
 */

#include "ros/ros.h"

#include <stdio.h>
#include <curl/curl.h>
#include <std_msgs/String.h>
#include <voldemort/callRobocop.h>


CURL *curl;
CURLcode res;
ros::Publisher resultPublisher;

size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata, ros::Publisher &publisher) {
	ROS_INFO("Calling callback.");
	std::string result;
	for (int c = 0; c<size*nmemb; c++) {
		result.push_back(ptr[c]);
	}

	std_msgs::String resultMsg;
	resultMsg.data = result;
	resultPublisher.publish(resultMsg);

	return size*nmemb;
}

void subscriberCallback(const voldemort::callRobocopPtr& msg) {
	ROS_INFO("Incoming request.");
	std::string url = msg->url;
	std::string agent = msg->agent;

	curl = curl_easy_init();
	if(curl) {
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

		if (!agent.empty()) {
			curl_easy_setopt(curl, CURLOPT_USERAGENT, agent.c_str());
		}

		curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);

		// TODO Check for debug-level
//		curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

		res = curl_easy_perform(curl);

		if(res != CURLE_OK) {
			ROS_ERROR("Failed performing request: %s\n", curl_easy_strerror(res));
		}

		curl_easy_cleanup(curl);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "call_robocop");
	ros::NodeHandle nh;

	resultPublisher = nh.advertise<std_msgs::String>("call_robocop_response", 1337);
	ros::Subscriber sub = nh.subscribe("call_robocop_request", 1, subscriberCallback);
	ROS_INFO("Waiting for requests.");
	ros::spin();
	return 0;
}
