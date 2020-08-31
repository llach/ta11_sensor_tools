#!/usr/bin/env python

import tf
import rospy
import roslib

from math import cos, sin
from tactile_msgs.msg import TactileState
from sensor_msgs.msg import ChannelFloat32

rospy.init_node( 'ta11_tactile_viz_test' )
rate = rospy.Rate(10)

topic = 'ta11_tactile_test'
publisher = rospy.Publisher( topic, TactileState, queue_size=5 )

v = 0
while not rospy.is_shutdown():

    ts = TactileState()
    ts.header.frame_id = "/base_link"
    ts.header.stamp = rospy.Time.now()

    left = ChannelFloat32()
    left.name = 'left_gripper_tactile'
    left.values.append(cos(v))

    right = ChannelFloat32()
    right.name = 'right_gripper_tactile'
    right.values.append(sin(v))

    ts.sensors.append(left)
    ts.sensors.append(right)

    publisher.publish( ts )

    v += .1
    rate.sleep()

