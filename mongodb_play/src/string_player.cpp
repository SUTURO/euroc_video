#include <ros/ros.h>
#include <mongo/client/dbclient.h>
#include <sstream>
#include <std_msgs/String.h>
#include <mongodb_play/string_player.h>

using namespace mongo;

StringPlayer::StringPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection):
  DBPlayer(nh, topic, db_address, database, collection)
{
  pub_ = nh.advertise<std_msgs::String>(topic, 1000);
};

void StringPlayer::pub_msg(BSONObj b)
{
  std_msgs::String msg;
  msg.data = b.getField("data").String();
  pub_.publish(msg);
};
