#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('chatter', String, queue_size=10)