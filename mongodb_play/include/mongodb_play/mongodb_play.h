#include "ros/ros.h"
#include <mongo/client/dbclient.h>
#include <actionlib/server/action_server.h>
#include <suturo_video_msgs/PlayAction.h>

using namespace mongo;

class MongoPlayer
{
public:
  /** Constructor
  */
  MongoPlayer(const ros::NodeHandle &node, std::string db_address, std::string database, std::string collection, std::string topic);

  /// Destructor
  ~MongoPlayer();

  /**
  * Plays
  */
  void play();


protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::ActionServer<suturo_video_msgs::PlayAction> action_server_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  suturo_video_msgs::PlayFeedback feedback_;
  suturo_video_msgs::PlayResult result_;
  suturo_video_msgs::PlayGoal goal_;

  void goalCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh);
  void cancelCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh);

};
