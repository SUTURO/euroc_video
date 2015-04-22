#include "ros/ros.h"
#include "mongo/client/dbclient.h"
#include "mongodb_play/mongodb_play.h"
#include "mongodb_play/db_player.h"

using namespace mongo;

DBPlayer::DBPlayer(std::string db_address, std::string database, std::string collection, std::string topic)
{
  std::string errmsg = "";
  //conn = new DBClientConnection(true);
  if (! conn.connect(db_address, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
  }

  std::stringstream ss;
  ss <<  database << "." << collection;
  db_coll = ss.str();
};

void DBPlayer::play(ros::Time system_time, ros::Time start_time, ros::Time end_time)
{
  auto_ptr<DBClientCursor> cursor = conn.query(db_coll, BSONObj());
  while (cursor->more())
    cout << cursor->next().toString() << endl;
};

void DBPlayer::pause()
{
  auto_ptr<DBClientCursor> cursor = conn.query(db_coll, BSONObj());
  while (cursor->more())
    cout << cursor->next().toString() << endl;
};

void DBPlayer::stop()
{};
