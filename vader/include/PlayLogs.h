#ifndef PLAYLOGS_H
#define PLAYLOGS_H

#include <ros/ros.h>
#include <suturo_video_msgs/PlayAction.h>
#include <actionlib/client/simple_action_client.h>

class PlayLogs
{
typedef actionlib::SimpleActionClient<suturo_video_msgs::PlayAction> Client;

public:
	PlayLogs();
	void play_logs(std::string msg_type, std::string database, std::string collection, std::string output_topic, ros::Time start, ros::Time end);

private:
    actionlib::SimpleActionClient<suturo_video_msgs::PlayAction>* client;
};

#endif // PLAYLOGS_H