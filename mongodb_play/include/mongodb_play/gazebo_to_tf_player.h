#include "ros/ros.h"
#include "mongo/client/dbclient.h"
#include "mongodb_play/db_player.h"
#include <geometry_msgs/Transform.h>

class GazeboToTFPlayer: public DBPlayer
{
public:
  GazeboToTFPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection);
  void pub_msg(mongo::BSONObj b);

private:
  int seq_;
  geometry_msgs::Transform read_transform(mongo::BSONObj b);
};
