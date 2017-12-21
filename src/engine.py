#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from kati.srv import *
import RPi.GPIO as GPIO
import time

pins = []


def callback_pwm(data):
    global pins
    plf = pins[0]
    plb = pins[1]
    prf = pins[2]
    prb = pins[3]
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    plf.ChangeDutyCycle(data.data)

def handle_stop_engine(*arg):
    global pins
    print "Stopping Engine"
    for pin in pins:
        pin.ChangeDutyCycle(0)
    b = "Stopping Engine"
    return b

def handle_forward(data):
    global pins
    a = data.a
    print "Forward -> !"
    pins[0].ChangeDutyCycle(a)
    pins[2].ChangeDutyCycle(a)
    b = "Forward -> !"
    return b

def engine():
    global pins
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('engine', anonymous=True)

    GPIO.setmode(GPIO.BCM)
    pin_left_for = 3
    pin_left_bac = 2
    pin_right_for = 15
    pin_right_bac = 14

    for pin_number in [pin_left_for, pin_left_bac, pin_right_for, pin_right_bac]:
        GPIO.setup(pin_number, GPIO.OUT)
        pins.append(GPIO.PWM(pin_number, 50))

    for pin in pins:
        pin.start(0)

    rospy.Service('stop_engine', StopEngine, handle_stop_engine)
    rospy.Service('forward', Forward, handle_forward)
    rospy.Subscriber('gpio_engine', Float32, callback_pwm)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    engine()
