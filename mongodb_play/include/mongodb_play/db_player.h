#include "ros/ros.h"
#include <mongo/client/dbclient.h>

class DBPlayer
{
public:
  DBPlayer(std::string db_address, std::string database, std::string collection, std::string topic);
  ~DBPlayer() {};
  virtual void play(ros::Time start_time, ros::Time end_time) {};
  virtual void pause() {};
  virtual void stop() {};

protected:
  std::string db_coll;
  mongo::DBClientConnection conn;
};
