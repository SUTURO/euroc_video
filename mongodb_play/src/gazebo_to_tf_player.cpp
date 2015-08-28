#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <sstream>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include "mongodb_play/gazebo_to_tf_player.h"

using namespace mongo;

GazeboToTFPlayer::GazeboToTFPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection):
  DBPlayer(nh, topic, db_address, database, collection)
{
  pub_ = nh.advertise<tf2_msgs::TFMessage>(topic, 1000);
};

void GazeboToTFPlayer::pub_msg(BSONObj b)
{
  tf2_msgs::TFMessage msg;
  geometry_msgs::TransformStamped transform_stamped;
  BSONObj pose_stamped = b.getField("posestamped").Obj();
  std::string name = b.getField("name").String();

  // set header
  BSONObj db_header = pose_stamped.getField("header").Obj();
  transform_stamped.header.seq = db_header.getField("seq").Int();
  transform_stamped.header.frame_id = db_header.getField("frame_id").String();
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.child_frame_id = "/" + name;
  transform_stamped.transform = read_transform(pose_stamped.getField("pose").Obj());
  msg.transforms.push_back(transform_stamped);

  pub_.publish(msg);
};

geometry_msgs::Transform GazeboToTFPlayer::read_transform(BSONObj b)
{
  geometry_msgs::Transform transform;
  BSONObj translation = b.getField("position").Obj();
  transform.translation.x = translation.getField("x").Double();
  transform.translation.y = translation.getField("y").Double();
  transform.translation.z = translation.getField("z").Double();
  BSONObj rotation = b.getField("orientation").Obj();
  transform.rotation.x = rotation.getField("x").Double();
  transform.rotation.y = rotation.getField("y").Double();
  transform.rotation.z = rotation.getField("z").Double();
  transform.rotation.w = rotation.getField("w").Double();
  return transform;
};
