#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mongo/client/dbclient.h>
#include "mongodb_play/db_player.h"

class TFPlayer: public DBPlayer
{
public:
  TFPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection);
  ~TFPlayer() {};
  void play(ros::Time start_time, ros::Time end_time);
  void stop();

private:
  int seq_;

};
