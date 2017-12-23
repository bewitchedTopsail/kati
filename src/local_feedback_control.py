#!/usr/bin/env python
import rospy
from kati.msg import Trajectorie
from kati.msg import SteeringCmds
import time

def callback(data, pub):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.p1)

    steeringCmgsMsg = SteeringCmds()
    steeringCmgsMsg.throttle = 80
    steeringCmgsMsg.curvature = 0
    pub.publish(steeringCmgsMsg)

def local_feedback_control():

    # Init
    rospy.init_node('local_feedback_control', anonymous=True)
    pub = rospy.Publisher('steering_cmds', SteeringCmds, queue_size=10)
    rospy.Subscriber('trajectorie',Trajectorie, callback, pub)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    local_feedback_control()
