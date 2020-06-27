#!/usr/bin/env python
# used 4 spaces/blanks
import rospy
import math
from sensor_msgs.msg import Range

#Float
global meters
global centimeters

def callback(data):
    rospy.loginfo(rospy.get_name() + " I heard %f", data.range)
    meters = data.range
    rospy.loginfo("%f meters", meters)
    printCentimeters(meters) 

def printCentimeters(value):
    centimeters = transformMetersToCentimeters(value)
    print str(centimeters) + " cm"

# meters to centimeters with 2 decimal places
def transformMetersToCentimeters(value):
    return round(value * 100, 2)
    
def ultrasound_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ultrasound_listener', anonymous=True)

    rospy.Subscriber("rangeMsg_topic", Range, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    ultrasound_listener()
