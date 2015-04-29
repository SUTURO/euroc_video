#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include "mongodb_play/tf_player.h"

using namespace mongo;

TFPlayer::TFPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection):
  DBPlayer(nh, topic, db_address, database, collection)
{
  pub_ = nh.advertise<tf2_msgs::TFMessage>(topic, 1000);
  seq_ = 0;
};

void TFPlayer::play(ros::Time start_time, ros::Time end_time)
{
  tf2_msgs::TFMessage msg;
  auto_ptr<DBClientCursor> cursor = conn_.query(db_coll_, BSONObj());
  while (cursor->more())
  {
    BSONObj p = cursor->next();
    vector<BSONElement> db_transforms = p["transforms"].Array();
    for (vector<BSONElement>::iterator it = db_transforms.begin(); it != db_transforms.end(); ++it)
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.seq = seq_;
      BSONObj db_header = it->Obj().getField("header").Obj();
      transformStamped.header.frame_id = db_header.getField("frame_id");
      transformStamped.child_frame_id = it->Obj().getField("child_frame_id").toString();

      msg.transforms.push_back(transformStamped);
      cout << msg << endl;
    }

    pub_.publish(msg);
    seq_++;
  }
};

void TFPlayer::stop()
{};
