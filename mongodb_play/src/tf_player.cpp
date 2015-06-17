#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <sstream>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include "mongodb_play/tf_player.h"

using namespace mongo;

TFPlayer::TFPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection):
  DBPlayer(nh, topic, db_address, database, collection)
{
  pub_ = nh.advertise<tf2_msgs::TFMessage>(topic, 1000);
};

void TFPlayer::pub_msg(BSONObj b)
{
  tf2_msgs::TFMessage msg;
  vector<BSONElement> db_transforms = b["transforms"].Array();

  for (vector<BSONElement>::iterator it = db_transforms.begin(); it != db_transforms.end(); ++it)
  {
    geometry_msgs::TransformStamped transformStamped;
    BSONObj db_header = it->Obj().getField("header").Obj();
    transformStamped.header.seq = db_header.getField("seq").Int();
    transformStamped.header.frame_id = db_header.getField("frame_id").String();
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.child_frame_id = it->Obj().getField("child_frame_id").String();
    transformStamped.transform = read_transform(it->Obj().getField("transform").Obj());
    msg.transforms.push_back(transformStamped);
  }

  pub_.publish(msg);
};

geometry_msgs::Transform TFPlayer::read_transform(BSONObj b)
{
  geometry_msgs::Transform transform;
  BSONObj translation = b.getField("translation").Obj();
  transform.translation.x = translation.getField("x").Double();
  transform.translation.y = translation.getField("y").Double();
  transform.translation.z = translation.getField("z").Double();
  BSONObj rotation = b.getField("rotation").Obj();
  transform.rotation.x = rotation.getField("x").Double();
  transform.rotation.y = rotation.getField("y").Double();
  transform.rotation.z = rotation.getField("z").Double();
  transform.rotation.w = rotation.getField("w").Double();
  return transform;
};
