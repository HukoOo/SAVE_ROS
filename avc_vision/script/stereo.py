#!/usr/bin/env python
import cv2
import sys
import numpy as np
import time
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def nothing(x):
    pass

def decode_fourcc(v):
        v = int(v)
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])
def main(args):
    # Create a black image, a window
    cv2.namedWindow('control')

    # create trackbars for color change
    cv2.createTrackbar('Gain','control',0,255,nothing)
    cv2.createTrackbar('Exposure','control',0,255,nothing)

    left_image_pub = rospy.Publisher("ocam/left_image",Image)
    right_image_pub = rospy.Publisher("ocam/right_image",Image)
    bridge = CvBridge()
    exposure=0.0
    gain=0.0
    capture_properties = ['CAP_PROP_POS_MSEC', 'CAP_PROP_POS_FRAMES', 'CAP_PROP_POS_AVI_RATIO',
              'CAP_PROP_FRAME_WIDTH', 'CAP_PROP_FRAME_HEIGHT', 'CAP_PROP_FPS', 'CAP_PROP_FOURCC',
              'CAP_PROP_FRAME_COUNT', 'CAP_PROP_FORMAT', 'CAP_PROP_MODE', 'CAP_PROP_BRIGHTNESS',
              'CAP_PROP_CONTRAST', 'CAP_PROP_SATURATION', 'CAP_PROP_HUE', 'CAP_PROP_GAIN',
              'CAP_PROP_EXPOSURE', 'CAP_PROP_CONVERT_RGB', 'CAP_PROP_WHITE_BALANCE_BLUE_U',
              'CAP_PROP_RECTIFICATION', 'CAP_PROP_MONOCHROME', 'CAP_PROP_SHARPNESS',
              'CAP_PROP_AUTO_EXPOSURE', 'CAP_PROP_GAMMA', 'CAP_PROP_TEMPERATURE', 'CAP_PROP_TRIGGER',
              'CAP_PROP_TRIGGER_DELAY', 'CAP_PROP_WHITE_BALANCE_RED_V', 'CAP_PROP_ZOOM',
              'CAP_PROP_FOCUS', 'CAP_PROP_GUID', 'CAP_PROP_ISO_SPEED', 'CAP_PROP_BACKLIGHT',
              'CAP_PROP_PAN', 'CAP_PROP_TILT', 'CAP_PROP_ROLL', 'CAP_PROP_IRIS', 'CAP_PROP_SETTINGS',
              'CAP_PROP_BUFFERSIZE', 'CAP_PROP_AUTOFOCUS']
    left_cam = cv2.VideoCapture()
    right_cam = cv2.VideoCapture()

    left_port = rospy.get_param("/ocam_stream/left_port")       
    right_port = rospy.get_param("/ocam_stream/right_port")

    left_cam.open(left_port)
    right_cam.open(right_port)

    print 'left_cam open:', left_cam.isOpened()
    print 'right_cam open:', right_cam.isOpened()

    # print camera.get(cv2.CAP_PROP_FOURCC)
    # C = camera.get(cv2.CAP_PROP_FOURCC)
    # print 'fourcc original:', decode_fourcc(C)

    # codec = 0x47504A4D # MJPG
    # codec = 844715353.0 # YUY2
    codec = 1196444237.0  # MJPG

    # print 'fourcc:', decode_fourcc(codec)
    left_cam.set(cv2.CAP_PROP_FOURCC, codec)
    left_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    left_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    left_cam.set(cv2.CAP_PROP_FPS, 60.0)
    left_cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)

    right_cam.set(cv2.CAP_PROP_FOURCC, codec)
    right_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    right_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    right_cam.set(cv2.CAP_PROP_FPS, 60.0)
    right_cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)

    print right_cam.get(cv2.CAP_PROP_FRAME_WIDTH)
    print right_cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print right_cam.get(cv2.CAP_PROP_FPS)
    C = right_cam.get(cv2.CAP_PROP_FOURCC)
    print 'fourcc:', decode_fourcc(C)

    print
    for i in xrange(38):
            print i, capture_properties[i], right_cam.get(i)

    while (1):

        
        # get current positions of four trackbars
        exposure = cv2.getTrackbarPos('Exposure','Control')
        gain = cv2.getTrackbarPos('Gain','Control')

        #right_cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 1.0)
        #right_cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 1.0)
        right_cam.set(cv2.CAP_PROP_GAIN, gain)
        right_cam.set(cv2.CAP_PROP_EXPOSURE, exposure)


        #left_cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 1.0)
        #left_cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 1.0)
        left_cam.set(cv2.CAP_PROP_GAIN, gain)
        left_cam.set(cv2.CAP_PROP_EXPOSURE, exposure)
        

        left_cam.grab()
        right_cam.grab()
        retval, left_im = left_cam.retrieve(0)
        retval, right_im = right_cam.retrieve(0)

        left_image_pub.publish(bridge.cv2_to_imgmsg(left_im, "bgr8"))
        right_image_pub.publish(bridge.cv2_to_imgmsg(right_im, "bgr8"))


    left_cam.release()
    right_cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    
    rospy.init_node('image_converter', anonymous=True)
    main(sys.argv)