#!/usr/bin/env python

#examples of valid message
#{
#    "mode" : basic,
#    "values" : {
#        "left" : "35",
#        "right" : "38"
#    }
#}

# $ export MY_MSG="data: '{\"mode\":\"basic\",\"values\":{\"left\":\"0\",\"right\":\"0\"}}'";rostopic pub -1 /motors std_msgs/String "$MY_MSG"
# $ export MY_MSG="data: '{\"mode\":\"basic\",\"values\":{\"left\":\"30\",\"right\":\"80\"}}'";rostopic pub -1 /motors std_msgs/String "$MY_MSG"

import RPi.GPIO as GPIO

import rospy
from std_msgs.msg import String
import json
import simplejson

ForwardLeft=32
BackwardLeft=29
ForwardRight=33
BackwardRight=31
PwmFrequency=50 #Hz

#gpio configuration
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ForwardLeft, GPIO.OUT)
GPIO.setup(BackwardLeft, GPIO.OUT)
GPIO.setup(ForwardRight, GPIO.OUT)
GPIO.setup(BackwardRight, GPIO.OUT)

fl = GPIO.PWM(ForwardLeft, PwmFrequency)
fr = GPIO.PWM(ForwardRight, PwmFrequency)
bl = GPIO.PWM(BackwardLeft, PwmFrequency)
br = GPIO.PWM(BackwardRight, PwmFrequency)
fl.start(0)
fr.start(0)
bl.start(0)
br.start(0)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    try:
        #parse the json
        jsonData = simplejson.loads(data.data)
        #check the mode 
        if jsonData["mode"].lower() == "basic":
            left = float(jsonData["values"]["left"])
            right = float(jsonData["values"]["right"])
            #drive motors
            if left > 0.0:
                fl.ChangeDutyCycle(left)
                bl.ChangeDutyCycle(0.0)
            else:
                fl.ChangeDutyCycle(0.0)
                bl.ChangeDutyCycle(-1 * left)
            if right > 0.0:
                fr.ChangeDutyCycle(right)
                br.ChangeDutyCycle(0.0)
            else:
                fr.ChangeDutyCycle(0.0)
                br.ChangeDutyCycle(-1 * right)
    except ValueError as e:
            rospy.logerr("Unable to parse json input ( %s ): %s", data.data, e)

def motor_listener():

    rospy.init_node('motor_listener', anonymous=True)
    rospy.Subscriber('motors', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    motor_listener()
