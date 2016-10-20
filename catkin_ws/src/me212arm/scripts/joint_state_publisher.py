#!/usr/bin/python

# 2.12 Lab 5 joint publisher from dynamixel
# Peter Yu Oct 2016

import dynamixel_msgs.msg
import sensor_msgs.msg
import message_filters
import rospy


rospy.init_node("dynamixel_joint_state_publisher")
joint1_sub = message_filters.Subscriber("/joint1_controller/state", dynamixel_msgs.msg.JointState)
joint2_sub = message_filters.Subscriber("/joint2_controller/state", dynamixel_msgs.msg.JointState)
joint_pub = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState,  queue_size=1) 

def joint_callback(j1, j2):
    #print j1, j2
    js = sensor_msgs.msg.JointState( header = j1.header, 
                                     name = ['joint1', 'joint2'],
                                     position = [j1.current_pos, j2.current_pos],
                                     velocity = [j1.velocity, j2.velocity],
                                     effort = [j1.load, j2.load] )
    joint_pub.publish(js)
    
ts = message_filters.ApproximateTimeSynchronizer([joint1_sub, joint2_sub], 20, 1)
ts.registerCallback(joint_callback)

rospy.spin()
