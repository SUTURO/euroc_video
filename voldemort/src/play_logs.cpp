#include <ros/ros.h>
#include <suturo_video_msgs/PlayAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_video_msgs::PlayAction> Client;

// using namespace std;

// Methode wird zum abpsielen von Daten gebraucht, Main ist nur für die Dummy Daten da
void play_logs(std::string msg_type, std::string database, std::string collection, std::string output_topic, ros::Time start, ros::Time end)
{        
        Client client("play_action", true);
        std::cout << "after client" << std::endl;
        // wait for complete client initialisation
        ros::WallDuration(0.5).sleep();
        // waiting for connection
        client.waitForServer();
        ROS_INFO("Connected to server, ready to play_logs!");

        suturo_video_msgs::PlayGoal goal;

        goal.msg_type = msg_type;
        goal.database = database;
        goal.collection = collection;
        goal.output_topic = output_topic;
        goal.start_time = start;
        goal.end_time = end;

        std::cout << "before send goal" << std::endl;

        client.sendGoal(goal);
}

int main(int argc, char** argv)
{       
        std::cout << "start" << std::endl;
        if (argc != 1)
        {
                ROS_INFO("DO NOT GAVE ARGUMENTEES, NOT NEADED, JUST FOR TEHSTHING");
                return 1;
        }
        std::cout << "before ros init" << std::endl;
        
        ros::init(argc, argv, "Play_Logs");

        // Dummy Data

        std::string msg_type = "std_msgs/String";
        std::string database = "strings4";
        std::string collection = "strings";
        std::string output_topic = "output";
        ros::Time start = {secs: 1435136553, nsecs: 0};
        ros::Time end = {secs: 1435136568, nsecs: 0};

        std::cout << "after data" << std::endl;

        play_logs(msg_type, database, collection, output_topic, start, end);
        
        return 0;
}