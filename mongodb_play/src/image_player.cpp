#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <sstream>
#include <sensor_msgs/Image.h>
#include "mongodb_play/image_player.h"

using namespace mongo;

ImagePlayer::ImagePlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection):
  DBPlayer(nh, topic, db_address, database, collection)
{
  pub_ = nh.advertise<sensor_msgs::Image>(topic, 1000);
};

void ImagePlayer::pub_msg(BSONObj b)
{
  sensor_msgs::Image msg;
  // Read header
  BSONObj db_header = b.getField("header").Obj();
  msg.header.seq = db_header.getField("seq").Int();
  msg.header.frame_id = db_header.getField("frame_id").String();
  msg.header.stamp = ros::Time::now();
  // read heigth, width
  msg.height = b.getField("height").Int();
  msg.width = b.getField("width").Int();
  // read stuff
  msg.encoding = b.getField("encoding").String();
  msg.is_bigendian = b.getField("is_bigendian").Int();
  msg.step = b.getField("step").Int();
  // read data
  vector<BSONElement> db_data = b["data"].Array();
  for (vector<BSONElement>::iterator it = db_data.begin(); it != db_data.end(); ++it)
  {
    msg.data.push_back(it->Int());
  }

  pub_.publish(msg);
};
