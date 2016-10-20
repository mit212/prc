#!/usr/bin/python

# 2.12 Lab 5 trajectory planning
# Peter Yu Oct 2016
import rospy
import planner
import std_msgs.msg, sensor_msgs.msg
import numpy as np

rospy.init_node("run_planning")

exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint2_pub = rospy.Publisher('/joint2_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)

use_real_arm = rospy.get_param('/real_arm', False)

if __name__=="__main__":
    radius = 0.05          # (meter)
    center = [0.2, 0.15]  # (x,z) meter
    
    robotjoints = rospy.wait_for_message('/joint_states', sensor_msgs.msg.JointState)
    q0 = robotjoints.position
    
    for theta in np.linspace(0, 4*np.pi):
        target_xz = [center[0] + radius * np.cos(theta) , center[1] + radius * np.sin(theta) ]
        q_sol = planner.ik(target_xz, q0)
        if q_sol is None:
            print 'no ik solution'
        else:
            print '(q_1,q_2)=', q_sol
            if use_real_arm:
                exec_joint1_pub.publish(std_msgs.msg.Float64(q_sol[0]))
                exec_joint2_pub.publish(std_msgs.msg.Float64(q_sol[1]))
            else:
                js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = q_sol)
                exec_joint_pub.publish(js)
            q0 = q_sol

        rospy.sleep(0.3)

