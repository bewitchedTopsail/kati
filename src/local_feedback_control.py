#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import time

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def local_feedback_control():

    # Init
    rospy.init_node('local_feedback_control', anonymous=True)
    rospy.Subscriber('path',String, callback)
    pub = rospy.Publisher('gpio_engine', Float32, queue_size=10)

    time.sleep(1)
    pub.publish(80)
    time.sleep(2)
    pub.publish(0)
    rospy.loginfo(rospy.get_caller_id() + "Sending")



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    local_feedback_control()
