#include <ros/ros.h>
#include <mongo/client/dbclient.h>
#include <actionlib/server/action_server.h>
#include <suturo_video_msgs/PlayAction.h>
#include <mongodb_play/string_player.h>
#include <mongodb_play/tf_player.h>
#include <mongodb_play/image_player.h>

class MongoPlayer
{
public:
  /** Constructor
  */
  MongoPlayer(const ros::NodeHandle &node, std::string db_address);

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
  std::string db_address_;

  std::map<std::string, actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle> goal_handles_;
  std::map<std::string, boost::shared_ptr<DBPlayer> > db_players_;

  void goalCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh);
  void cancelCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh);

};
