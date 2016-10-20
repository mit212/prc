#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('object_detection', anonymous=True)

# Publisher for publishing pyramid marker in rviz
vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10) 

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()  

# Get the camera calibration parameter for the rectified image
msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None) 
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]

fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]

def main():
    useHSV   = False 
    useDepth = False
    if not useHSV:
        # Task 1
        
        # 1. initialize an OpenCV window
        cv2.namedWindow("OpenCV_View")
        
        # 2. set callback func for mouse hover event
        cv2.setMouseCallback("OpenCV_View", cvWindowMouseCallBackFunc)
        
        # 3. subscribe to image
        rospy.Subscriber('/camera/rgb/image_rect_color', Image, rosImageVizCallback)
    else:
        if not useDepth:
            # Task 2 Detect object using HSV
            #    Subscribe to RGB images
            rospy.Subscriber('/camera/rgb/image_rect_color', Image, rosHSVProcessCallBack)
        else:
            # Task 3: Use Kinect depth data
            #    Subscribe to both RGB and Depth images with a Synchronizer
            image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
            depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

            ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
            ts.registerCallback(rosRGBDCallBack)

    rospy.spin()

# Task 1 callback for ROS image
def rosImageVizCallback(msg):
    # 1. convert ROS image to opencv format
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # 2. visualize it in a cv window
    cv2.imshow("OpenCV_View", cv_image)
    cv2.waitKey(3)

# Task 1 callback for mouse event
def cvWindowMouseCallBackFunc(event, xp, yp, flags, param):
    print 'In cvWindowMouseCallBackFunc: (xp, yp)=', xp, yp  # xp, yp is the mouse location in the window
    # 1. Set the object to 2 meters away from camera
    zc = 2.0
    # 2. Visualize the pyramid
    showPyramid(xp, yp, zc, 10, 10)

# Task 2 callback
def rosHSVProcessCallBack(msg):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        
    contours, mask_image = HSVObjectDetection(cv_image)
    
    for cnt in contours:
        # Find a bounding box of detected region
        #  xp, yp are the coordinate of the top left corner of the bounding rectangle
        #  w, h are the width and height of the bounding rectangle
        xp,yp,w,h = cv2.boundingRect(cnt)  
        
        # Set the object to 2 meters away from camera
        zc = 2    
        
        # Draw the bounding rectangle
        cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255], 2)
        
        centerx, centery = xp+w/2, yp+h/2
        showPyramid(centerx, centery, zc, w, h)
    

# Task 2 object detection code
def HSVObjectDetection(cv_image, toPrint = True):
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # define range of red color in HSV
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv_image, lower_red, upper_red)   ##
    mask_eroded         = cv2.erode(mask, None, iterations = 3)  ##
    mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 10)  ##
    
    if toPrint:
        print 'hsv', hsv_image[240][320] # the center point hsv
        
    showImageInCVWindow(cv_image, mask_eroded, mask_eroded_dilated)
    contours,hierarchy = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    return contours, mask_eroded_dilated

# Task 3 callback
def rosRGBDCallBack(rgb_data, depth_data):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
        cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
    except CvBridgeError as e:
        print(e)

    contours, mask_image = HSVObjectDetection(cv_image, toPrint = False)

    for cnt in contours:
        xp,yp,w,h = cv2.boundingRect(cnt)
        
        # Get depth value from depth image, need to make sure the value is in the normal range 0.1-10 meter
        if not math.isnan(cv_depthimage2[int(yp)][int(xp)]) and cv_depthimage2[int(yp)][int(xp)] > 0.1 and cv_depthimage2[int(yp)][int(xp)] < 10.0:
            zc = cv_depthimage2[int(yp)][int(xp)]
            #print 'zc', zc
        else:
            continue
            
        centerx, centery = xp+w/2, yp+h/2
        cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255],2)
        
        showPyramid(centerx, centery, zc, w, h)

def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    ## 
    xn = (xp - cx) / fx
    yn = (yp - cy) / fy
    xc = xn * zc
    yc = yn * zc
    return (xc,yc,zc)

def showImageInCVWindow(cv_image, mask_erode_image, mask_image):
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask = mask_image)
    
    # Draw a cross at the center of the image
    cv2.line(cv_image, (320, 235), (320, 245), (255,0,0))
    cv2.line(cv_image, (325, 240), (315, 240), (255,0,0))
    
    # Show the images
    cv2.imshow('OpenCV_Original', cv_image)
    cv2.imshow('OpenCV_Mask_Erode', mask_erode_image)
    cv2.imshow('OpenCV_Mask_Dilate', mask_image)
    cv2.imshow('OpenCV_View', res)
    cv2.waitKey(3)

# Create a pyramid using 4 triangles
def showPyramid(xp, yp, zc, w, h):
    # X1-X4 are the 4 corner points of the base of the pyramid
    X1 = getXYZ(xp-w/2, yp-h/2, zc, fx, fy, cx, cy)
    X2 = getXYZ(xp-w/2, yp+h/2, zc, fx, fy, cx, cy)
    X3 = getXYZ(xp+w/2, yp+h/2, zc, fx, fy, cx, cy)
    X4 = getXYZ(xp+w/2, yp-h/2, zc, fx, fy, cx, cy)
    vis_pub.publish(createTriangleListMarker(1, [X1, X2, X3, X4], rgba = [1,0,0,1], frame_id = '/camera'))

# Create a list of Triangle markers for visualization
def createTriangleListMarker(marker_id, points, rgba, frame_id = '/camera'):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.TRIANGLE_LIST
    marker.scale = Vector3(1,1,1)
    marker.id = marker_id
    
    n = len(points)
    
    if rgba is not None:
        marker.color = ColorRGBA(*rgba)
        
    o = Point(0,0,0)
    for i in xrange(n):
        p = Point(*points[i])
        marker.points.append(p)
        p = Point(*points[(i+1)%4])
        marker.points.append(p)
        marker.points.append(o)
        
    marker.pose = poselist2pose([0,0,0,0,0,0,1])
    return marker

def poselist2pose(poselist):
    return Pose(Point(*poselist[0:3]), Quaternion(*poselist[3:7]))

if __name__=='__main__':
    main()
    
