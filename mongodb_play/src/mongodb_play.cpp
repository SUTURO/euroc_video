#include <ros/ros.h>
#include <mongo/client/dbclient.h>
#include <suturo_video_msgs/PlayAction.h>
#include <mongodb_play/mongodb_play.h>

using namespace mongo;

MongoPlayer::MongoPlayer(const ros::NodeHandle &node, std::string db_address):
  nh_(node),
  action_server_(nh_, "play_action",
    boost::bind(&MongoPlayer::goalCallback, this, _1),
    boost::bind(&MongoPlayer::cancelCallback, this, _1),
    false)
{
  db_address_ = db_address;
  action_server_.start();
  ROS_INFO("Started action server");
};

MongoPlayer::~MongoPlayer()
{};

void MongoPlayer::goalCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh)
{
  ROS_INFO("Goal received: %s", gh.getGoalID().id.c_str());

  // Create db_player
  boost::shared_ptr<DBPlayer>  dbpl_ptr;
  if (gh.getGoal()->msg_type == "std_msgs/String")
  {
    ROS_INFO("Using StringPlayer");
    dbpl_ptr.reset( new StringPlayer(nh_, gh.getGoal()->output_topic,db_address_, gh.getGoal()->database, gh.getGoal()->collection) );
  }
  else if (gh.getGoal()->msg_type == "tf2_msgs/TFMessage")
  {
    ROS_INFO("Using TFPlayer");
    dbpl_ptr.reset( new TFPlayer(nh_, gh.getGoal()->output_topic,db_address_, gh.getGoal()->database, gh.getGoal()->collection) );
  }
  else if (gh.getGoal()->msg_type == "sensor_msgs/Image")
  {
    ROS_INFO("Using ImagePlayer");
    dbpl_ptr.reset( new ImagePlayer(nh_, gh.getGoal()->output_topic,db_address_, gh.getGoal()->database, gh.getGoal()->collection) );
  }
  else if (gh.getGoal()->msg_type == "sensor_msgs/JointState")
  {
    ROS_INFO("Using JointStatePlayer");
    dbpl_ptr.reset( new JointStatePlayer(nh_, gh.getGoal()->output_topic,db_address_, gh.getGoal()->database, gh.getGoal()->collection) );
  }
  else
  {
    ROS_INFO("No player for type %s", gh.getGoal()->msg_type.c_str());
    gh.setRejected();
    return;
  }

  db_players_[gh.getGoalID().id] = dbpl_ptr;

  // Accept goal
  goal_handles_[gh.getGoalID().id] = gh;
  gh.setAccepted();
  ROS_INFO("Goal accepted: %s", gh.getGoalID().id.c_str());
  cout << "Start: " << ros::Time::now().nsec << endl;
  dbpl_ptr->play(gh.getGoal()->start_time, gh.getGoal()->end_time);
};

void MongoPlayer::cancelCallback(actionlib::ActionServer<suturo_video_msgs::PlayAction>::GoalHandle gh)
{
  ROS_INFO("Goal cancel requested: %s", gh.getGoalID().id.c_str());

  // Check if the goal is there
  if(goal_handles_.find(gh.getGoalID().id) != goal_handles_.end())
  {
    goal_handles_.erase(gh.getGoalID().id);
    db_players_[gh.getGoalID().id]->stop();
    db_players_.erase(gh.getGoalID().id);
    gh.setCanceled();
    ROS_INFO("Goal canceled: %s", gh.getGoalID().id.c_str());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mongodb_play_node", ros::init_options::AnonymousName);
  ros::NodeHandle node;

  MongoPlayer mongoPlayer(node, "localhost");
  ros::spin();
  return 0;
}