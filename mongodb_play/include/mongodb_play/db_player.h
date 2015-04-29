#include "ros/ros.h"
#include <std_msgs/String.h>
#include <mongo/client/dbclient.h>

class DBPlayer
{
public:
  DBPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection);
  ~DBPlayer() {};
  virtual void play(ros::Time start_time, ros::Time end_time) {};
  virtual void stop() {};

protected:
  std::string db_coll_;
  mongo::DBClientConnection conn_;
  ros::Publisher pub_;
};
