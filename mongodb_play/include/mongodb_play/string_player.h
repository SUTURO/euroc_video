#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include "mongodb_play/db_player.h"

class StringPlayer: public DBPlayer
{
public:
  StringPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection);
  void pub_msg(mongo::BSONObj b);
};