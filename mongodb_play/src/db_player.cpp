#include "ros/ros.h"
#include <std_msgs/String.h>
#include "mongo/client/dbclient.h"
#include "mongodb_play/db_player.h"

using namespace mongo;

DBPlayer::DBPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection)
{
  std::string errmsg = "";
  if (! conn_.connect(db_address, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
  }

  std::stringstream ss;
  ss <<  database << "." << collection;
  db_coll_ = ss.str();
};
