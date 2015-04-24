#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <sstream>
#include <string>
#include "mongodb_play/tf_player.h"

using namespace mongo;

TFPlayer::TFPlayer(std::string db_address, std::string database, std::string collection, std::string topic):
  DBPlayer(db_address, database, collection, topic)
{

};

void TFPlayer::play(ros::Time start_time, ros::Time end_time)
{
  auto_ptr<DBClientCursor> cursor = conn.query(db_coll, BSONObj());
    while (cursor->more())
    cout << cursor->next().toString() << endl;
};

void TFPlayer::pause()
{
  auto_ptr<DBClientCursor> cursor = conn.query(db_coll, BSONObj());
    while (cursor->more())
    cout << cursor->next().toString() << endl;
};

void TFPlayer::stop()
{};
