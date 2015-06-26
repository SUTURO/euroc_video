#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mongo/client/dbclient.h>
#include <mongo/util/time_support.h>
#include <iostream>
#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <mongodb_play/db_player.h>

using namespace mongo;

DBPlayer::DBPlayer(ros::NodeHandle nh, std::string topic, std::string db_address, std::string database, std::string collection)
{
  std::string errmsg = "";
  if (! conn_.connect(db_address, errmsg)) {
    ROS_ERROR("Failed to connect to MongoDB: %s", errmsg.c_str());
  }

  std::stringstream ss;
  ss <<  database << "." << collection;
  db_coll_ = ss.str();
  m_stopped_ = false;
};

DBPlayer::~DBPlayer()
{};

void DBPlayer::play(ros::Time start_time, ros::Time end_time)
{
  cout << "Starting thread" << endl;
  boost::thread player(&DBPlayer::play_thread, this, start_time, end_time);
  cout << "Started thread" << endl;
};

void DBPlayer::pause()
{
  set_paused(true);
  cout << "Paused" << endl;
};

void DBPlayer::unpause()
{
  set_paused(false);
  cout << "Unpaused" << endl;
};

void DBPlayer::stop()
{
  {
      boost::unique_lock<boost::mutex> lock(m_stop_mutex_);
      m_stopped_ = true;
  }
  {
      boost::unique_lock<boost::mutex> lock(m_timer_mutex_);
      m_blocked_ = false;
  }
  m_pulse_timer_.notify_all();
  cout << "Stopped" << endl;
};

void DBPlayer::finish()
{
  cout << "Finished goal" << endl;
};

void DBPlayer::play_thread(ros::Time start_time, ros::Time end_time)
{
  cout << "Started play" << endl;
  ros::Time time_zero = ros::Time::now();
  ros::Duration time_offset;
  bool offset_is_set = false;
  auto_ptr<DBClientCursor> cursor = conn_.query(db_coll_, BSONObj());
  BSONObj p;
  ros::Time t;
  uint32_t secs;
  uint32_t nsecs;
  BSONElement date;

  // iterate through the messages
  while (cursor->more())
  {
    p = cursor->next();

    // get time of message
    date = p.getField("__recorded");
    if (date.type() == NumberDouble)
    {
      t = ros::Time().fromSec(date.Double());
    }
    else if (date.type() == Date)
    {
      secs = date.Date().millis / 1000;
      nsecs = (date.Date().millis % 1000) * 1000000;
      t = ros::Time(secs, nsecs);
    }

    // check if time is after start time
    if (t < start_time)
      continue;

    // check if time is after end time
    if ((end_time.sec > 1) && (end_time.nsec > 1) && (t > end_time))
    {
      cout << "Reached end time: " << end_time << std::endl;
      break;
    }
    // set offset
    if (!offset_is_set)
    {
      time_offset = time_zero - t;
      offset_is_set = true;
      cout << "db time in secs: " << t.sec << "." << t.nsec << endl;
      cout << "ros time in secs: " << time_zero.sec << "." << time_zero.nsec <<endl;
      cout << "offset : " << time_offset.sec << "." << time_offset.nsec << endl;
    }

    // wait until time of message
    {
        boost::unique_lock<boost::mutex> lock(m_timer_mutex_);
        m_blocked_ = true;
    }
    boost::thread timer(&DBPlayer::block_until, this, t + time_offset);
    wait_for_timer();

    // stop if stop signal was sent
    {
        boost::unique_lock<boost::mutex> lock(m_stop_mutex_);
        if (m_stopped_)
        {
          cout << "Stopped pub" << std::endl;
          break;
        }
    }

    pub_msg(p);
  }

  finish();
};

void DBPlayer::set_paused(bool value)
{
  {
      boost::unique_lock<boost::mutex> lock(m_pause_mutex_);
      m_pause_ = value;
  }

  m_pause_changed_.notify_all();
};

void DBPlayer::block_while_paused()
{
    boost::unique_lock<boost::mutex> lock(m_pause_mutex_);
    while(m_pause_)
    {
        m_pause_changed_.wait(lock);
    }
};

void DBPlayer::block_until(ros::Time t)
{
  {
      boost::unique_lock<boost::mutex> lock(m_timer_mutex_);
      m_blocked_ = true;
  }
  ros::Time::sleepUntil(t);
  {
      boost::unique_lock<boost::mutex> lock(m_timer_mutex_);
      m_blocked_ = false;
  }
  m_pulse_timer_.notify_all();
};

void DBPlayer::wait_for_timer()
{
    boost::unique_lock<boost::mutex> lock(m_timer_mutex_);
    while(m_blocked_)
    {
        m_pulse_timer_.wait(lock);
    }
};
