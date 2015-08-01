#include <PlayLogs.h>

PlayLogs::PlayLogs(void)
{
    client = new actionlib::SimpleActionClient<suturo_video_msgs::PlayAction>("play_action", true);

    std::cout << "after client" << std::endl;
    // wait for complete client initialisation
    ros::WallDuration(0.5).sleep();
    // waiting for connection
    int waitTimer = 0;
    while (!client->isServerConnected() && waitTimer < 5){
        ros::WallDuration(0.5).sleep();
        std::cout << "Waiting for server..." << std::endl;
        waitTimer += 1;
    }

    if (waitTimer == 5){
        std::cout << "Connection to Server failed!" << std::endl;
    } else {
        ROS_INFO("Connected to server, ready to play_logs!");
    }
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
