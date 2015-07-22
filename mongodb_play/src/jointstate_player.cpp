#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include "mongodb_play/jointstate_player.h"

using namespace mongo;

JointStatePlayer::JointStatePlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection):
  DBPlayer(nh, topic, db_address, database, collection)
{
  pub_ = nh.advertise<sensor_msgs::JointState>(topic, 1000);
};

void JointStatePlayer::pub_msg(BSONObj b)
{
  sensor_msgs::JointState msg;
  // Read header
  BSONObj db_header = b.getField("header").Obj();
  msg.header.seq = db_header.getField("seq").Int();
  msg.header.frame_id = db_header.getField("frame_id").String();
  msg.header.stamp = ros::Time::now();
  // read name
  vector<BSONElement> db_name = b["name"].Array();
  for (vector<BSONElement>::iterator it = db_name.begin(); it != db_name.end(); ++it)
  {
    msg.name.push_back(it->String());
  }
  // read name
  vector<BSONElement> db_position = b["position"].Array();
  for (vector<BSONElement>::iterator it = db_position.begin(); it != db_position.end(); ++it)
  {
    msg.position.push_back(it->Double());
  }
  // read name
  vector<BSONElement> db_velocity = b["velocity"].Array();
  for (vector<BSONElement>::iterator it = db_velocity.begin(); it != db_velocity.end(); ++it)
  {
    msg.velocity.push_back(it->Double());
  }
  // read name
  vector<BSONElement> db_effort = b["effort"].Array();
  for (vector<BSONElement>::iterator it = db_effort.begin(); it != db_effort.end(); ++it)
  {
    msg.effort.push_back(it->Double());
  }


  pub_.publish(msg);
};
