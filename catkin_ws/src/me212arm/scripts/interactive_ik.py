#!/usr/bin/env python


import roslib; roslib.load_manifest("interactive_markers")
import rospy
import tf
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from tf.broadcaster import TransformBroadcaster
from interactive_markers.interactive_marker_server import *
    
from threading import Thread, Lock
import sensor_msgs.msg, std_msgs.msg
from me212helper.helper import poselist2pose, transformPose, pubFrame, pose2poselist
from me212helper.marker_helper import createInteractiveMarker, createMoveControlsXZ

import planner

use_real_arm = rospy.get_param('/real_arm', False)

exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)

exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
exec_joint2_pub = rospy.Publisher('/joint2_controller/command', std_msgs.msg.Float64, queue_size=1)

mutex = Lock()
def frameCallback( msg ):
    global br
    global currentTargetPose, currentTargetPoseDirty
    global listener
    time = rospy.Time.now()
    
    
    mutex.acquire()
    pubFrame(br, pose2poselist(currentTargetPose), 'target_pose', 'arm_base', npub = 1)
    
    mutex.release()
    

def solveIk(target_pose):
    global q0
    if q0 is None:
        robotjoints = rospy.wait_for_message('/joint_states', sensor_msgs.msg.JointState)
        q0 = robotjoints.position[0:2]
    
    q_sol = planner.ik([target_pose[0], target_pose[2]], q0)
    if q_sol is None:
        print 'no ik solution'
        return
    print '(q_1,q_2)=', q_sol
    
    if use_real_arm:
        exec_joint1_pub.publish(std_msgs.msg.Float64(q_sol[0]))
        exec_joint2_pub.publish(std_msgs.msg.Float64(q_sol[1]))
    else:
        js = sensor_msgs.msg.JointState(name=['joint1', 'joint2'], position = q_sol)
        exec_joint_pub.publish(js)
        
    
    q0 = q_sol


def processFeedback(feedback):
    global currentTargetPose, currentTargetPoseDirty
    p = feedback.pose.position
    o = feedback.pose.orientation
    print feedback.marker_name + ' is now at (%f, %f, %f)' % (p.x, p.y, p.z)
    
    mutex.acquire()
    currentTargetPose = feedback.pose
    
    #t = transformPose(lr, pose2poselist(feedback.pose), 'map', 'arm_base')
    t = pose2poselist(feedback.pose)
    t[2] -= 0.25
    
    solveIk(pose2poselist(currentTargetPose))
    mutex.release()
    

rospy.init_node("interactive_ik_server")
br = TransformBroadcaster()
lr = tf.TransformListener()

if __name__=="__main__":
    
    global currentTargetPose, currentTargetPoseDirty
    global q0
    q0 = None

    rospy.sleep(0.1)
    currentTargetPose = Pose()
    currentTargetPoseDirty = False
    
    # create an interactive marker server on the topic namespace ik_interactive
    server = InteractiveMarkerServer("ik_interactive")
    
    # create an interactive marker for TargetPose
    pose_link2 = [0, 0, 0.235, 0, 0, 0, 1]
    pose_arm_base = transformPose(lr, pose_link2, 'link2', 'arm_base')
    
    pose_arm_base[3] = 0
    pose_arm_base[4] = 0
    pose_arm_base[5] = 0
    pose_arm_base[6] = 1
    
    currentTargetPose = poselist2pose(pose_arm_base)
    
    int_marker = createInteractiveMarker('target_pose', *pose_arm_base, frame_id='arm_base')
    int_marker.controls.extend(createMoveControlsXZ())
    server.insert(int_marker, processFeedback)

    rospy.Timer(rospy.Duration(0.1), frameCallback)

    server.applyChanges()
    print 'ready'
    rospy.spin()

