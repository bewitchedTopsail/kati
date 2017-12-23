#!/usr/bin/env python

import rospy
from kati.srv import *
from kati.msg import SteeringCmds
import RPi.GPIO as GPIO
import numpy as np

pins = []
s = 10
L = 2*11.5+2*2.5*np.pi
T = 30
v_max_l = 9*L/T
v_max_r = 8*L/T

def callback(steeringCmdMsg):
    global s
    global pins
    global v_max_l
    global v_max_r
    global v_max

    v_des = steeringCmdMsg.throttle
    curv_des = steeringCmdMsg.curvature
    v_l = (1 - s * curv_des / 2) * v_des
    v_r = (1 + s * curv_des / 2) * v_des
    print "v_l des: " + str(v_l)
    print "v_r des: " + str(v_r)

    # Normieren auf v_max
    v_l = v_l  / v_max_l * 100
    v_r = v_r  / v_max_r * 100
    print "v_l 0...100: " + str(v_l)
    print "v_r 0...100: " + str(v_r)

    v_max_des = np.maximum(v_r, v_l)
    if v_max_des > 100:
        v_r = v_r / v_max_des*100
        v_l = v_l / v_max_des*100
        print " speed to high for curve, reducing speed, v_max_des = " + str(v_max_des)

    print "v_l 0...100: " + str(v_l)
    print "v_r 0...100: " + str(v_r)



    plf = pins[0]
    plb = pins[1]
    prf = pins[2]
    prb = pins[3]
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s, %s', steeringCmdMsg.throttle, steeringCmdMsg.curvature)


    if curv_des == 0:
        if steeringCmdMsg.throttle > 0:
            plf.ChangeDutyCycle(v_l)
            prf.ChangeDutyCycle(v_r)
            plb.ChangeDutyCycle(0)
            prb.ChangeDutyCycle(0)
        else:
            plf.ChangeDutyCycle(0)
            prf.ChangeDutyCycle(0)
            plb.ChangeDutyCycle(-v_l)
            prb.ChangeDutyCycle(-v_r)
    elif curv_des < 0:
        plf.ChangeDutyCycle(100)
        prf.ChangeDutyCycle(0)
        plb.ChangeDutyCycle(0)
        prb.ChangeDutyCycle(100)
    elif curv_des > 0:
        plf.ChangeDutyCycle(0)
        prf.ChangeDutyCycle(100)
        plb.ChangeDutyCycle(100)
        prb.ChangeDutyCycle(0)
    else:

        print  "No correct inputs"

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
    pins[0].ChangeDutyCycle(50*9/8)
    pins[2].ChangeDutyCycle(100)
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
    rospy.Subscriber('steering_cmds', SteeringCmds,callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    engine()