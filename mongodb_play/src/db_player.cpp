#include "ros/ros.h"
#include "mongo/client/dbclient.h"
#include "mongodb_play/db_player.h"

using namespace mongo;

DBPlayer::DBPlayer(std::string db_address, std::string database, std::string collection, std::string topic)
{
  std::string errmsg = "";
  if (! conn.connect(db_address, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
  }

  std::stringstream ss;
  ss <<  database << "." << collection;
  db_coll = ss.str();
};
