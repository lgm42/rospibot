#!/usr/bin/env python

from ina219 import INA219
from ina219 import DeviceRangeError
import rospy
from std_msgs.msg import String
import json

SHUNT_OHMS = 0.1

def read():
    pub = rospy.Publisher('ina219', String, queue_size=10)
    ledPublisher = rospy.Publisher('leds', String, queue_size=1, latch=True)

    rospy.init_node('ina219Reader', anonymous=True)
    rate = rospy.Rate(0.2) #hz

    ina = INA219(SHUNT_OHMS)
    ina.configure()
    while not rospy.is_shutdown():
        try:
            #TODO : low pass filter
            led = {}
            led["mode"] = "basic"
            led["values"] = {}
            led["values"]["led"] = "3"

            output = {}
            output['voltage'] = float("{0:.2f}".format(ina.voltage()))
            output['current'] = float("{0:.2f}".format(ina.current()))
            output['power'] = float("{0:.2f}".format(ina.power()))
            #output['shunt_voltage'] = "{0:.2f}".format(ina.shunt_voltage())

            #data interpretation
            if output['current'] < 0.0:
                output['charging'] = 'true'
                output['battery_level'] = 'N/A'
                led["values"]["color"] = "00FF00"
            else:
                output['charging'] = 'false'
                output['battery_level'] = "{0:.2f}".format(max(min(170.91 * output['voltage'] - 567.75, 100.0), 0.0))
                led["values"]["color"] = "000000"

            rospy.loginfo(json.dumps(output))
            pub.publish(json.dumps(output))
            
            #drive led
            ledPublisher.publish(json.dumps(led))

        except DeviceRangeError as e:
            # Current out of device range with specified shunt resister
            rospy.logerr(e)

        rate.sleep()

if __name__ == '__main__':
    try:
        read()
    except rospy.ROSInterruptException:
        pass
