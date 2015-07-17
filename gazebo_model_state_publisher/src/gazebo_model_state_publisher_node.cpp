#include "ros/ros.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <gazebo_model_state_publisher/NamedPoseStamped.h>

#include <iostream>
#include <sys/time.h>
#include <boost/unordered_map.hpp>

ros::Publisher pub;
std::vector<std::string> ignored_objects;
boost::unordered_map<std::string, long long> last_updated;
struct timeval tp;

void callback(ConstPosesStampedPtr &_msg)
{
  /* std::cout << _msg->DebugString(); */

  gazebo::msgs::Time gz_time = _msg->time();
  gettimeofday(&tp, NULL);
  long long current_time = (long long) tp.tv_sec * 1000 + tp.tv_usec / 1000;

  for(int i = 0; i < _msg->pose_size(); i++) {
    gazebo::msgs::Pose gz_pose = _msg->pose(i);

    if(std::find(ignored_objects.begin(),
                 ignored_objects.end(),
                 gz_pose.name()) != ignored_objects.end() ||
       current_time - last_updated[gz_pose.name()] < 500) {
      // Skip model update
      continue;
    }

    geometry_msgs::PoseStamped ros_pose_stamped;
    geometry_msgs::Pose ros_pose;

    std_msgs::Header ros_header;
    ros_header.frame_id = "/base_footprint";
    ros_header.stamp = ros::Time(gz_time.sec(), gz_time.nsec());
    ros_pose_stamped.header = ros_header;

    gazebo::msgs::Vector3d gz_position = gz_pose.position();
    geometry_msgs::Point ros_position;
    ros_position.x = gz_position.x();
    ros_position.y = gz_position.y();
    ros_position.z = gz_position.z();
    ros_pose.position = ros_position;

    gazebo::msgs::Quaternion gz_orientation = gz_pose.orientation();
    geometry_msgs::Quaternion ros_orientation;
    ros_orientation.x = gz_orientation.x();
    ros_orientation.y = gz_orientation.y();
    ros_orientation.z = gz_orientation.z();
    ros_orientation.w = gz_orientation.w();
    ros_pose.orientation = ros_orientation;

    ros_pose_stamped.pose = ros_pose;

    gazebo_model_state_publisher::NamedPoseStamped ros_named_pose_stamped;
    ros_named_pose_stamped.name = gz_pose.name();
    ros_named_pose_stamped.posestamped = ros_pose_stamped;

    pub.publish(ros_named_pose_stamped);

    last_updated[gz_pose.name()] = current_time;

  }
}

int main(int _argc, char *_argv[])
{

  ignored_objects.assign(_argv + 1, _argv + _argc);

  ros::init(_argc, _argv, "gazebo_model_state_publisher");
  gazebo::load(_argc, _argv);

  ros::NodeHandle n;

  pub = n.advertise<gazebo_model_state_publisher::NamedPoseStamped>("/gazebo/model_poses", 1000);

  ros::Rate loop_rate(10);


  gazebo::run();

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", callback);

  ros::spin();

  gazebo::fini();
}
