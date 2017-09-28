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

class image_converter:
    def __init__(self,args):
        self.left_image_pub = rospy.Publisher("ocam/left_image",Image)
        self.right_image_pub = rospy.Publisher("ocam/right_image",Image)
        self.bridge = CvBridge()
        self.exposure=500.0
        self.gain=100.0
        self.capture_properties = ['CAP_PROP_POS_MSEC', 'CAP_PROP_POS_FRAMES', 'CAP_PROP_POS_AVI_RATIO',
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
        self.left_cam = cv2.VideoCapture()
        self.right_cam = cv2.VideoCapture()

        left_port = rospy.get_param("/ocam_stream/left_port")    	
        right_port = rospy.get_param("/ocam_stream/right_port")

        self.left_cam.open(left_port)
        self.right_cam.open(right_port)

        print 'left_cam open:', self.left_cam.isOpened()
        print 'right_cam open:', self.right_cam.isOpened()

        # print camera.get(cv2.CAP_PROP_FOURCC)
        # C = camera.get(cv2.CAP_PROP_FOURCC)
        # print 'fourcc original:', decode_fourcc(C)

        # codec = 0x47504A4D # MJPG
        # codec = 844715353.0 # YUY2
        self.codec = 1196444237.0  # MJPG

        # print 'fourcc:', decode_fourcc(codec)
        self.left_cam.set(cv2.CAP_PROP_FOURCC, self.codec)
        self.left_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.left_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.left_cam.set(cv2.CAP_PROP_FPS, 60.0)
        #self.left_cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)

        self.right_cam.set(cv2.CAP_PROP_FOURCC, self.codec)
        self.right_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.right_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.right_cam.set(cv2.CAP_PROP_FPS, 60.0)
        #self.right_cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)

        #self.right_cam.set(cv2.CAP_PROP_GAIN, self.gain)
        #self.right_cam.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

        #self.left_cam.set(cv2.CAP_PROP_GAIN, self.gain)
        #self.left_cam.set(cv2.CAP_PROP_EXPOSURE, self.exposure)

        print self.right_cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        print self.right_cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print self.right_cam.get(cv2.CAP_PROP_FPS)
        C = self.right_cam.get(cv2.CAP_PROP_FOURCC)
        print 'fourcc:', self.decode_fourcc(C)

        print
        for i in xrange(38):
            print i, self.capture_properties[i], self.right_cam.get(i)

    def track_bar(self):
    	cv2.namedWindow('Control')
        cv2.createTrackbar('Exposure','Control',0,625,nothing)
        cv2.createTrackbar('Gain','Control',0,255,nothing)

    def decode_fourcc(self,v):
        v = int(v)
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

    def pubImages(self):
        # create trackbars for color change
        #self.track_bar()

        while (1):
            
            '''
            # get current positions of four trackbars
            self.exposure = cv2.getTrackbarPos('Exposure','Control')
            self.gain = cv2.getTrackbarPos('Gain','Control')

            #self.right_cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 1.0)
            #self.right_cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 1.0)
            self.right_cam.set(cv2.CAP_PROP_GAIN, self.gain)
            self.right_cam.set(cv2.CAP_PROP_EXPOSURE, self.exposure)


            #self.left_cam.set(cv2.CAP_PROP_WHITE_BALANCE_RED_V, 1.0)
            #self.left_cam.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 1.0)
            self.left_cam.set(cv2.CAP_PROP_GAIN, self.gain)
            self.left_cam.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
            '''

            self.left_cam.grab()
            self.right_cam.grab()
            retval, left_im = self.left_cam.retrieve(0)
            retval, right_im = self.right_cam.retrieve(0)

            self.left_image_pub.publish(self.bridge.cv2_to_imgmsg(left_im, "bgr8"))
            self.right_image_pub.publish(self.bridge.cv2_to_imgmsg(right_im, "bgr8"))
			

        self.left_cam.release()
        self.right_cam.release()

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter(args)
    ic.pubImages()


if __name__ == '__main__':
    main(sys.argv)