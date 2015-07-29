#!/usr/bin/env python
import subprocess
import threading
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('gazebo_model_state_startup', anonymous=True)
    rospy.wait_for_message("joint_states", JointState)
    #os.system('roslaunch gazebo_model_state_publisher start_publisher_and_logger.launch')
    subprocess.Popen('rosrun suturo_planning_executive mongodb_log /gazebo/model_poses', shell=True)
    subprocess.Popen('rosrun gazebo_model_state_publisher gazebo_model_state_publisher_node', shell=True)
    rospy.spin()
