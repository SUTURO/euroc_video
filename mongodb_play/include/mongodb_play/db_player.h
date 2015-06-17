#ifndef DBPLAYER_H
#define DBPLAYER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <mongo/client/dbclient.h>

class DBPlayer
{
public:
  DBPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection);
  ~DBPlayer();
  void play(ros::Time start_time, ros::Time end_time);
  void pause();
  void unpause();
  virtual void pub_msg(mongo::BSONObj b) {};
  void stop();

protected:
  std::string db_coll_;
  mongo::DBClientConnection conn_;
  ros::Publisher pub_;

private:
  void finish();
  void play_thread(ros::Time start_time, ros::Time end_time);
  void set_paused(bool value);
  void block_while_paused();
  boost::mutex m_pause_mutex_;
  boost::condition_variable m_pause_changed_;
  bool m_pause_;
  void block_until(ros::Time t);
  void wait_for_timer();
  boost::mutex m_timer_mutex_;
  boost::condition_variable m_pulse_timer_;
  bool m_blocked_;
  boost::mutex m_stop_mutex_;
  bool m_stopped_;
};

#endif
