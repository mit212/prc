#!/usr/bin/python

# 2.12 Lab 5 RRT for obstacle-free trajectory planning (adapted from Steve LaValle's code)
# Peter Yu Oct 2016

import rospy
import random
from math import sqrt,cos,sin,atan2
from planner import fk, fk1, joint_limits, a1, a2
import numpy as np
import sensor_msgs.msg
from std_msgs.msg import Float64
from me212helper.marker_helper import createLineStripMarker, createPointMarker, createSphereMarker
from visualization_msgs.msg import Marker
import collision
import tkMessageBox

# parameters
XDIM = a1+a2 + 0.1
ZDIM = a1+a2 + 0.1
EPSILON = 0.03        # (rad)
TARGET_RADIUS = 0.02  # (meter)
NUMNODES = 5000
NIter = 10000

obstacle_segs = [ [[0.25,0.15], [0.4,0.2]], [[0.25,0.15-0.01], [0.4,0.2-0.01]] ]  # line segs ((x1,z1)--(x2,z2))
#obstacle_segs = []  # no obstacles
target_x = [0.3, 0.1]
q0 = [-0.01, 0.0]

def dist(p1, p2):
    return sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]))

def step_from_toward(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)

class Node:
    def __init__(self, q, parent_id = None): # q: joint position (q1, q2), x: location of the TCP (x, z)
        self.q = q
        self.parent_id = parent_id

def find_nearest_node(nodes, q):
    min_index = 0
    for i, p in enumerate(nodes):
        if dist(q, p.q) < dist(q, nodes[min_index].q):
            min_index = i
    return min_index

def in_workspace(q):
    x = fk(q)
    return x[0] >= 0 and x[0] <= XDIM and x[1] >= 0 and x[1] <= ZDIM 

# trace from "node" back to the root of the tree represented by "nodes"
def backtrace(nodes, node):
    plan = []
    curr_node = node
    while True:
        plan.append(curr_node.q)
        if curr_node.parent_id is None:
            break
        curr_node = nodes[ curr_node.parent_id ]
        
    plan.reverse()
    return plan

def rrt(target_x, q0, NIter = 10000, pub = None, vis_pub= None):
    nodes = [ Node(q0) ]
    start_x = fk(q0)
    
    if vis_pub is not None:
        vis_pub.publish(createSphereMarker(2, namespace="", pose = [target_x[0], 0, target_x[1], 0, 0, 0 ,1], rgba=(0,0,1,1), frame_id = '/arm_base'))
        rospy.sleep(0.2)
        vis_pub.publish(createSphereMarker(3, namespace="", pose = [start_x[0], 0, start_x[1], 0, 0, 0 ,1], rgba=(1,0,1,1), frame_id = '/arm_base'))
        rospy.sleep(0.2)
    
    for i in xrange(NIter):
        if i % 100 == 0:
            print 'iteration:', i
            rospy.sleep(0.01)
        # pick a node in work space randomly
        q_rand = [ np.random.uniform(joint_limits[0][0], joint_limits[0][1]), 
                   np.random.uniform(joint_limits[1][0], joint_limits[1][1]) ]
        nearest_node_index = find_nearest_node (nodes, q_rand)
        new_q = step_from_toward(nodes[nearest_node_index].q, q_rand)
        
        #print 'new_q', new_q
        #print 'in_workspace(new_q)', in_workspace(new_q)
        #print 'in_collision(new_q)', in_collision(new_q)
        
        if pub is not None:
            js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = (new_q[0], new_q[1]))
            pub.publish(js)
        
        
        if in_workspace(new_q) and not collision.in_collision(new_q, obstacle_segs):
            if vis_pub is not None:
                xz = fk(new_q)
                xz_old = fk(nodes[nearest_node_index].q)
                vis_pub.publish(createPointMarker([[xz[0], 0, xz[1]]], i+6, namespace="", rgba=(0,1,0,1), frame_id = '/arm_base'))
                vis_pub.publish(createLineStripMarker([[xz_old[0], 0, xz_old[1]] , [xz[0], 0, xz[1]]], 
                        marker_id = i+200000, rgba = (0,0.5,0,1), pose=[0,0,0,0,0,0,1], frame_id = '/arm_base'))
            
            new_node = Node(new_q, nearest_node_index)
            nodes.append(new_node)
            
            if dist(fk(new_q), target_x) < TARGET_RADIUS:
                plan = backtrace(nodes, new_node)
                
                return plan
    
    return None # no plan found

def exec_joints(exec_real_pub, q):
    exec_real_pub[0].publish(Float64(q[0]))
    exec_real_pub[1].publish(Float64(q[1]))

def main():
    rospy.init_node("test_rrt")
    
    # initiate publishers
    exec_virtual_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)
    exec_real_pubs = [rospy.Publisher('/joint1_controller/command', Float64, queue_size=1) ,
                      rospy.Publisher('/joint2_controller/command', Float64, queue_size=1) ]
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=100) 
    rospy.sleep(0.5)
    
    # get whether to use real arm
    use_real_arm = rospy.get_param('/real_arm', False)    
    
    # visualizing obstacles
    vis_pub.publish(Marker(action=3)) # delete all markers
    rospy.sleep(0.5)
    for iseg, seg in enumerate(obstacle_segs):
        vis_pub.publish(createLineStripMarker([[seg[0][0], 0, seg[0][1]] , [seg[1][0], 0, seg[1][1]]], 
                    marker_id = iseg+100000, rgba = (1,0,0,1), pose=[0,0,0,0,0,0,1], frame_id = '/arm_base'))

    # back to the starting pose
    if use_real_arm:
        exec_joints(exec_real_pubs, q0)
        rospy.sleep(0.5)
    
    # run rrt
    print 'target_x', target_x
    print 'q0', q0
    plan = rrt(target_x = target_x, q0 = q0, NIter = NIter, pub = exec_virtual_pub, vis_pub= vis_pub)
    if plan is None:
        print 'no plan found in %d iterations' % NIter
        return
    
    print 'found 1 plan', plan
    
    # execute it
    result = tkMessageBox.askquestion("RRT Plan", "A plan is found, execute?", icon='warning')
    if result == 'yes':
        for p in plan:
            if not use_real_arm:
                js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = (p[0], p[1]))
                exec_virtual_pub.publish(js)
            else:
                exec_joints(exec_real_pubs, p)
            
            rospy.sleep(0.1)

# if python says run, then we should run
if __name__ == '__main__':
    main()

