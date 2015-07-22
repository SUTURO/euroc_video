#!/usr/bin/env python
import rospy
#from tf2_msgs.msg import TFMessage
import tf

def talker():
    rospy.init_node('tftalker', anonymous=True)
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransform(
            (0,0,0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "name" ,
            "frame")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
