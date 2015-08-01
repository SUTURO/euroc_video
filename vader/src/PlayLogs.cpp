#include <PlayLogs.h>

PlayLogs::PlayLogs(void)
{
    client = new actionlib::SimpleActionClient<suturo_video_msgs::PlayAction>("play_action", true);

    std::cout << "after client" << std::endl;
    // wait for complete client initialisation
    ros::WallDuration(0.5).sleep();
    // waiting for connection
    client->waitForServer();
    ROS_INFO("Connected to server, ready to play_logs!");
}

void PlayLogs::play_logs(std::string msg_type, std::string database, std::string collection, std::string output_topic, ros::Time start, ros::Time end)
{        
        suturo_video_msgs::PlayGoal goal;

        goal.msg_type = msg_type;
        goal.database = database;
        goal.collection = collection;
        goal.output_topic = collection;
        goal.start_time = start;
        goal.end_time = end;

        std::cout << "before send goal" << std::endl;

        client->sendGoal(goal);
}