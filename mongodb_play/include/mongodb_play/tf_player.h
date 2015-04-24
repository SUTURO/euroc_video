#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include "mongodb_play/db_player.h"

class TFPlayer: public DBPlayer
{
public:
  TFPlayer(std::string db_address, std::string database, std::string collection, std::string topic);
  ~TFPlayer() {};
  void play(ros::Time start_time, ros::Time end_time);
  void pause();
  void stop();

};
