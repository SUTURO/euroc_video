#include <ros/ros.h>
#include <mongo/client/dbclient.h>
#include <suturo_video_msgs/PlayAction.h>
#include "mongodb_play/mongodb_play.h"
#include "mongodb_play/db_player.h"

using namespace mongo;

MongoPlayer::MongoPlayer(const ros::NodeHandle &node, std::string db_address, std::string database, std::string collection, std::string topic):
  nh_(node),
  action_server_(nh_, "play_action",
    boost::bind(&MongoPlayer::goalCallback, this, _1),
    boost::bind(&MongoPlayer::cancelCallback, this, _1),
    false)
{
  action_server_.start();

  DBPlayer dbpl(db_address, database, collection, topic);
  dbpl.pause();
  cout << "End" << endl;
};

MongoPlayer::~MongoPlayer()
{};

void MongoPlayer::goalCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh)
{

};

void MongoPlayer::cancelCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh)
{

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mongodb_play_node", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  MongoPlayer mongoPlayer(node, "localhost", "mydb", "JointStates", "topic");
  return 0;
}
