#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket
import sys
sys.path.append('/home/jjm/workspace/ROS/avc_ws/devel/lib/python2.7/dist-packages')

import rospy, rospkg, math
import time, copy
import ctypes
from std_msgs.msg import Int8, Bool, Float32, Float64
from geometry_msgs.msg import Point32
from teb_local_planner.msg import SKKU_CenterLaneArray
from teb_local_planner.msg import SKKU_LaneArray
# lidar
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
# from jung-min
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

host = '127.0.0.1'
#host = '192.168.1.1'
# host = 'skku-Alienware-X51-R2'
port = 21560

# 01234.678  x 6 + 5 = 54 + 5 = 59
# 01234.678, 01234.678, 01234.678, 01234.678, 01234.678, 01234.678
# -1234.678
# 012345678

def StringToFloat(str_data):
    if '-' in str_data:
        index = str_data.find('-')
        new_data = str_data[index:9]
        return float(new_data)
    else:
        return float(str_data)

def StringToUInt(str_data):
    try:
        return abs(int(str_data))
    except ValueError:
        print 'value error: ', str_data
        return 0



class TCP_Client:
    def __init__(self, host, port):
        # self.mission_size = 1
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        address = (host, port)
        try:
            print('Attempts to connect.')
            self.tcp_client.connect(address)
        except Exception as err:
            print ('%s:%s' % address), err
            sys.exit('Unable to connect to TCP/IP.') 
        print('Connected to TCP/IP.')

    def send_data(self, data_str):
        #print 'test'
        self.tcp_client.send(data_str)
        #conn, addr = self.tcp_client.accept()     # Establish connection with client.
        #conn.send(data_str)
        print "sender: ", time.time()
        #conn.close()



#######################################################



def Float_to_TCP_String(digit, float_data):
    # form = '.'+str(digit)+'f'
    # str_data = format(float_data, form)
    str_data = str(round(float_data, digit))
    return str_data # str(float)

def get_number_before_divmod(quotient, remainder):
    return quotient*256 + remainder

class TcpManager:
    def __init__(self):
        rospy.init_node("tcp_skku_sender")
        my_tcp = TCP_Client(host, port)

        # ros subscribe
        rospy.Subscriber('/tcp_mission', Int8, self.CB_tcp_mission)
        self.mission = 0
        rospy.Subscriber('/tcp_lat', Float64, self.CB_tcp_lat)
        self.lat = 0.0
        rospy.Subscriber('/tcp_lon', Float64, self.CB_tcp_lon)
        self.lon = 0.0
        rospy.Subscriber('/tcp_heading', Float32, self.CB_tcp_heading)
        self.heading = 0.0
        rospy.Subscriber('/tcp_is_sim', Bool, self.CB_tcp_is_sim)
        self.is_sim = True

        # %.2f, S-curve enter
        rospy.Subscriber('Sensor/center_points',MarkerArray, self.CB_s_enter)
        self.s_enter = [3.0, 0.0] # [m]
        # %.2f, error from lane
        rospy.Subscriber('/obstacle_converter/center_lane', SKKU_CenterLaneArray, self.CB_lane_err)
        self.lane_err = 0.0 # [m]
        # heading
        rospy.Subscriber('/lane_array', SKKU_LaneArray, self.CB_lane_array)
        self.lane_interval= 0.0 #
        self.s_start= [0.0, 0.0]

        # %.2f, mission4 car position
        rospy.Subscriber('/obstacle_converter/tracking_target', Point32, self.CB_tracking_target)
        self.target_pos = [3.0, 0.0] # [m]
        # %.3f, mission4 acceleration
        rospy.Subscriber('acceleration/car_acc', Float32, self.CB_acc)
        self.accel = 0.0 # [m/s^2]
        # %.2f, mission1 inclination of obs
        rospy.Subscriber('Sensor/filtered_objects',MarkerArray, self.CB_incl_obs)
        self.incl_obs = 0.0
        # %.2f, mission3 inclination of building
        rospy.Subscriber('Sensor/side_object_pose',MarkerArray, self.CB_building)
        self.incl_building = 0.0
        self.left_building_incl=0.0
        self.right_building_incl=0.0
        self.left_building_dist=0.0
        self.right_building_dist=0.0
        # %.9f, lat, lon, heading
        rospy.Subscriber("/cRio_lat", Float64, self.CB_lat)
        rospy.Subscriber("/cRio_lon", Float64, self.CB_lon)
        rospy.Subscriber("/cRio_heading", Float32, self.CB_heading)
        self.bag_lat = 0.0
        self.bag_lon = 0.0
        self.bag_heading = 0.0
        # len=unkown, lidar raw
        rospy.Subscriber('/Sensor/clustered_points', PointCloud2, self.CB_lidar)
        self.lidar_array = []
        self.lidar_buff = []
        self.lidar_array_used = True # Semaphore

        r = rospy.Rate(100)
        # while not rospy.is_shutdown():
        while True:
            if 1: #not self.lidar_array_used:
                # Subscribe
                # print '\n ============================================'
                # self.mission.data, lat, lon, heading, is_sim = my_tcp.recv_data()
                # self.mission_pub.publish(self.mission)

                # Publish
                # %d, len=10, time
                t_str = str(int(time.time()))
                data_without_len = t_str+','
                # %.2f, S-curve enter
                data_without_len += Float_to_TCP_String(2, self.s_enter[0])+',' # x
                data_without_len += Float_to_TCP_String(2, self.s_enter[1])+',' # y
                # %.2f, error from lane
                data_without_len += Float_to_TCP_String(2, self.lane_err)+','
                # heading
                data_without_len += Float_to_TCP_String(2, self.lane_interval)+','
                # %.2f, mission4 car position
                data_without_len += Float_to_TCP_String(2, self.target_pos[0])+',' # x
                data_without_len += Float_to_TCP_String(2, self.target_pos[1])+',' # y
                # %.3f, mission4 acceleration
                data_without_len += Float_to_TCP_String(3, self.accel)+','
                # %.2f, mission1 inclination of obs
                data_without_len += Float_to_TCP_String(2, self.incl_obs)+','
                # %.2f, mission3 inclination of building
                data_without_len += Float_to_TCP_String(2, self.incl_building)+','
                # %.9f, lat, lon, heading
                if self.is_sim:
                    data_without_len += Float_to_TCP_String(2, self.bag_lat)+','
                    data_without_len += Float_to_TCP_String(2, self.bag_lon)+','
                    data_without_len += Float_to_TCP_String(2, self.bag_heading)+','
                else:
                    data_without_len += Float_to_TCP_String(2, lat)+','
                    data_without_len += Float_to_TCP_String(2, lon)+','
                    data_without_len += Float_to_TCP_String(2, heading)+','
                
                data_without_len += Float_to_TCP_String(2, self.s_start[0])+',' # left
                data_without_len += Float_to_TCP_String(2, self.s_start[1])+',' # right
                '''
                print self.left_building_incl
                print self.left_building_dist
                print self.right_building_incl
                print self.right_building_dist
                '''
                data_without_len += Float_to_TCP_String(2, self.left_building_incl)+',' 
                data_without_len += Float_to_TCP_String(2, self.right_building_incl)+',' 
                data_without_len += Float_to_TCP_String(2, self.left_building_dist)+','                         
                data_without_len += Float_to_TCP_String(2, self.right_building_dist)+','                 

                # len=unkown, lidar raw
                # (x*20)//256, %256, (y*20)//256, %256
                if(len(self.lidar_array)>0):
                    for point in self.lidar_array:
                        x = point[0]*20
                        y = (point[1]+10)*20
                        if (x>0) and (x<800) and (y>0) and (y<400):
                            x_quotient, x_remainder = divmod(x, 256) # float, float 
                            y_quotient, y_remainder = divmod(y, 256) # float, float 
                            data_without_len += chr(int(x_quotient))
                            data_without_len += chr(int(x_remainder))
                            data_without_len += chr(int(y_quotient))
                            data_without_len += chr(int(y_remainder))          
                        
                # data length
                n = len(data_without_len)
                quotient, remainder = divmod(n, 256)
                data = chr(quotient)+chr(remainder)+data_without_len # ascii
                # print 'len: ', get_number_before_divmod(quotient, remainder), ' DATA: ', data
                # print 'Time: '+t_str+' sec / Mission:', self.mission.data
                #print 'before'
                
                my_tcp.send_data(data)
                self.lidar_array = []
                #print 'after'
                r.sleep()


    def CB_tcp_mission(self, mission):
        self.mission = mission.data
    def CB_tcp_lat(self, lat):
        self.lat = lat.data
    def CB_tcp_lon(self, lon):
        self.lon = lon.data
    def CB_tcp_heading(self, heading):
        self.heading = heading.data
    def CB_tcp_is_sim(self, sim):
        self.is_sim = sim.data
    def CB_lane_array(self, lane_array):
        self.lane_interval = lane_array.interval
        self.s_start[0] = lane_array.lane[3].left
        self.s_start[1] = lane_array.lane[3].right

    def CB_s_enter(self, data):
        #point_num = len(data.markers)
        #MESSAGE = str(point_num)+","
        for marker in data.markers:
            self.s_enter[0] = marker.pose.position.x
            self.s_enter[1] = marker.pose.position.y
            break
            #MESSAGE = MESSAGE+str(self.s_enter[0])
            #MESSAGE = MESSAGE+","
            #MESSAGE = MESSAGE+str(self.s_enter[1])
            #MESSAGE = MESSAGE+","
    def CB_lane_err(self, err):
        self.lane_err = err.center[0]
    def CB_tracking_target(self, target_info):
        self.target_pos[0] = target_info.x
        self.target_pos[1] = target_info.y
    def CB_acc(self, acc):
        self.accel = acc.data
    def CB_incl_obs(self, data):
        #point_num = len(data.markers)
        #MESSAGE = str(point_num) + ","
        for marker in data.markers:
            #max_x = marker.points[0].x
            #max_y = marker.points[0].y
            #min_x = marker.points[1].x
            #min_y = marker.points[1].y
            self.incl_obs = float(marker.text)
            break
            #MESSAGE = MESSAGE + str(max_x)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(max_y)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(min_x)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(min_y)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + angle
            #MESSAGE = MESSAGE + ","
    def CB_building(self, data):
        #point_num = len(data.markers)
        #MESSAGE = str(point_num) + ","
        '''
        self.left_building_incl = 0.0
        self.left_building_dist = 0.0
        self.right_building_incl = 0.0
        self.right_building_dist = 0.0
        
        for marker in data.markers:
            ns = marker.ns
            pt1_x = marker.points[0].x
            pt1_y = marker.points[0].y
            pt2_x = marker.points[1].x
            pt2_y = marker.points[1].y
            self.incl_building = math.atan2(pt2_y-pt1_y, pt2_x-pt1_x)
            if ns == 'left_line':
                self.left_building_incl = math.atan2(pt2_y-pt1_y, pt2_x-pt1_x)
                self.left_building_dist = float(marker.text)
            if ns == 'right_line':
                self.right_building_incl = math.atan2(pt2_y-pt1_y, pt2_x-pt1_x)
                self.right_building_dist = float(marker.text)
            break
        '''
        if len(data.markers)==2:
            self.left_building_incl = float(data.markers[0].text)
            self.left_building_dist = math.sqrt(data.markers[0].points[0].y*data.markers[0].points[0].y+data.markers[0].points[0].x*data.markers[0].points[0].x)
            self.right_building_incl = float(data.markers[1].text)
            self.right_building_dist = math.sqrt(data.markers[1].points[0].y*data.markers[1].points[0].y+data.markers[1].points[0].x*data.markers[1].points[0].x)
            
        elif len(data.markers)==1:
            if data.markers[0].ns=='left_line':
                self.left_building_incl = float(data.markers[0].text)
                self.left_building_dist = math.sqrt(data.markers[0].points[0].y*data.markers[0].points[0].y+data.markers[0].points[0].x*data.markers[0].points[0].x)
                self.right_building_dist = 0.0
                self.right_building_incl = 0.0

            elif data.markers[0].ns=='right_line':
                self.left_building_dist = 0.0
                self.left_building_incl = 0.0
                self.right_building_incl = float(data.markers[0].text)
                self.right_building_dist = math.sqrt(data.markers[0].points[0].y*data.markers[0].points[0].y+data.markers[0].points[0].x*data.markers[0].points[0].x)
        elif len(data.markers)==0:
                self.left_building_dist = 0.0
                self.left_building_incl = 0.0 
                self.right_building_dist = 0.0
                self.right_building_incl = 0.0
        print data
        print len(data.markers)
        print self.left_building_incl
        print self.left_building_dist
        print self.right_building_incl
        print self.right_building_dist
            #dist = marker.text
            #MESSAGE = MESSAGE + ns
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(pt1_x)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(pt1_y)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(pt2_x)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + str(pt2_y)
            #MESSAGE = MESSAGE + ","
            #MESSAGE = MESSAGE + dist
            #MESSAGE = MESSAGE + ","
    def CB_lat(self, latitude):
        self.bag_lat = latitude.data
    def CB_lon(self, longitude):
        self.bag_lon = longitude.data
    def CB_heading(self, radian):
        self.bag_heading = radian.data
        # self.bag_heading = math.radians(90.) - radian.data
    def CB_lidar(self, data):
        height = data.height / 2.0
        middle_x = data.width / 2.0
        if (height >= data.height) or (middle_x >= data.width): return -1
        
        if self.lidar_array_used:
            self.lidar_array = []
            for point in pc2.read_points(data, skip_nans=True):
                self.lidar_array.append((point[0], point[1]))
            self.lidar_array_used = False
        else:
            self.lidar_array = copy.deepcopy(self.lidar_buff)
            self.lidar_buff = []
            for point in pc2.read_points(data, skip_nans=True):
                self.lidar_buff.append((point[0], point[1]))


if __name__ == '__main__':
    try:
        m = TcpManager()
    except rospy.ROSInterruptException:
        pass
