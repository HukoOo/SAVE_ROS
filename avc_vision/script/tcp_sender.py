#!/usr/bin/env python
import socket

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024



def center_cb(data):
    point_num = len(data.markers)
    MESSAGE = str(point_num)+","
    for marker in data.markers:
        pt_x = marker.pose.position.x
        pt_y = marker.pose.position.y

        MESSAGE = MESSAGE+str(pt_x)
        MESSAGE = MESSAGE+","
        MESSAGE = MESSAGE+str(pt_y)
        MESSAGE = MESSAGE+","

    print MESSAGE



def side_cb(data):
    point_num = len(data.markers)
    MESSAGE = str(point_num) + ","
    for marker in data.markers:
        ns = marker.ns
        pt1_x = marker.points[0].x
        pt1_y = marker.points[0].y
        pt2_x = marker.points[1].x
        pt2_y = marker.points[1].y
        dist = marker.text

        MESSAGE = MESSAGE + ns
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(pt1_x)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(pt1_y)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(pt2_x)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(pt2_y)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + dist
        MESSAGE = MESSAGE + ","
    print MESSAGE

def front_cb(data):
    point_num = len(data.markers)
    MESSAGE = str(point_num) + ","
    for marker in data.markers:

        max_x = marker.points[0].x
        max_y = marker.points[0].y
        min_x = marker.points[1].x
        min_y = marker.points[1].y
        angle = marker.text

        MESSAGE = MESSAGE + str(max_x)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(max_y)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(min_x)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + str(min_y)
        MESSAGE = MESSAGE + ","
        MESSAGE = MESSAGE + angle
        MESSAGE = MESSAGE + ","
    print MESSAGE

def marker_subscriber():
    rospy.init_node('marker_subscriber')
    rospy.Subscriber('Sensor/center_points',MarkerArray, center_cb)
    rospy.Subscriber('Sensor/side_object_pose',MarkerArray, side_cb)
    rospy.Subscriber('Sensor/filtered_objects',MarkerArray, front_cb)
    rospy.spin()

if __name__=='__main__':
    marker_subscriber()
