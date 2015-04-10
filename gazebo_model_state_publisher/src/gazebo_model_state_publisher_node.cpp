/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

#include <NamedPoseStamped.h>

#include "ros/ros.h"

#include <iostream>

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstPosesStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();

  ::gazebo::msg::Time time = _msg->time();

  for(int i = 0; i < _msg->pose_size(); i++) {
    ::gazebo::msg::Pose pose _msg->pose(i);
    ::std::string pose.name();

    ::gazebo::msgs::Vector3d position = pose.position();
    double position_x = position.x();
    double position_y = position.y();
    double position_z = position.z();

    ::gazebo::msgs::Quaternion orientation = pose.orientation();
    double orientation_x = orientation.x();
    double orientation_y = orientation.y();
    double orientation_z = orientation.z();
    double orientation_w = orientation.w();

    NamedPoseStamped named_pose;
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

  ros::init(_argc, _argv, "gazebo_model_state_publisher");
  gazebo::load(_argc, _argv);

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<NamedPoseStamped>("/gazebo/model_states", 1000);

  ros::Rate loop_rate(10);


  gazebo::run();

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", cb);

  ros::spin();

  gazebo::fini();
}
