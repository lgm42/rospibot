#!/usr/bin/python3.5

#examples of valid message
#{
#    "mode" : basic,
#    "values" : { 
#       "led" : "0",
#       "color" : "FF0033"
#     }
#}

# $ export MY_MSG="data: '{\"mode\":\"basic\",\"values\":{\"led\":\"0\",\"color\":\"FF00FF\"}}'";rostopic pub -1 /leds std_msgs/String "$MY_MSG"
# $ export MY_MSG="data: '{\"mode\":\"basic\",\"values\":{\"led\":\"2\",\"color\":\"FF0000\"}}'";rostopic pub -1 /leds std_msgs/String "$MY_MSG"

import board
import neopixel
import rospy
from std_msgs.msg import String
import json
import simplejson

def updateLeds(leds):
    for index, item in enumerate(leds):
        print(index, item)
        pixels[index] = (((item >> 16) & 0xFF), ((item >> 8) & 0xFF), (item & 0xFF))
    pixels.show()

def updateLed(led, color):
    pixels[led] = (((color >> 16) & 0xFF), ((color >> 8) & 0xFF), (color & 0xFF))

rospy.loginfo("Initializing leds")

# LED strip configuration:
LED_COUNT      = 4      # Number of LED pixels.
LED_PIN        = 10      # GPIO pin connected to the pixels 
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 60     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
LED_ORDER      = neopixel.GRB
rospy.init_node('leds_listener', anonymous=True)

rospy.loginfo("Initializing leds")

# Create NeoPixel object with appropriate configuration.
#strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
# Intialize the library (must be called once before other functions).
#strip.begin()

pixels = neopixel.NeoPixel(board.D10, LED_COUNT, brightness=0.2, auto_write=False, pixel_order=LED_ORDER)
 

ledColors = [0, 0, 0, 0]
updateLeds(ledColors)

rospy.loginfo("Initializing leds ok")

def callback(data):
    #print data._connection_header
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    try:
        #parse the json
        jsonData = simplejson.loads(data.data)
        #check the mode 
        if jsonData["mode"].lower() == "basic":
            led = int(jsonData["values"]["led"])
            color = int(jsonData["values"]["color"], 16)
            print("led:", led, "color", color)
            ledColors[led] = color
            #updateLeds(ledColors)
            updateLed(led, color)
            pixels.show()

    except ValueError as e:
            rospy.logerr("Unable to parse json input ( %s ): %s", data.data, e)

def leds_listener():

    rospy.Subscriber('leds', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        leds_listener()
    except rospy.ROSInterruptException:
        pass
