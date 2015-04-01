#!/usr/bin/env python

import roslib
roslib.load_manifest('clam_controller')

import time
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('make_cobra_pose', anonymous=True)
    
    publisher = rospy.Publisher('/elbow_roll_controller/command', Float64, queue_size=None)
    print 'Sending command'        
    publisher.publish(1.0)
