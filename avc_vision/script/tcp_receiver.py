#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket, time
import sys, copy
import rospy
from std_msgs.msg import Int8, Bool, Float64, Float32

host = '192.168.1.1'
port = 21570

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



class TCP_Server:
    def __init__(self, host, port):
        # self.mission_size = 1
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        address = (host, port)
        self.tcp_server.bind(address)
        self.tcp_server.listen(1)
        print 'Server listening....'
        self.conn, addr = self.tcp_server.accept()
        print self.conn, addr
        
    def recv_data(self):
        print "reciver: ", time.time()
        
        length_raw = self.conn.recv(4).decode("ascii")
        try:
            length = int(length_raw)
        except ValueError:
            length = 0
        # print length
        if length > 0:
            recv = self.conn.recv(length).decode("ascii")
            # print recv+'__end__'
            recv_split = recv.split(",")
            # mission, lat, lon, heading
            mission = int(StringToFloat(recv_split[0]))
            lat = StringToFloat(recv_split[1])
            lon = StringToFloat(recv_split[2])
            heading = StringToFloat(recv_split[3])
            is_sim = bool(recv_split[3])
            #conn.close()
            #print " ----end if"
            return mission, lat, lon, heading, is_sim
        else:
            #conn.close()
            #print " ----end else"
            return 0, 0.0, 0.0, 0.0, True


class TcpManager:
    def __init__(self):
        rospy.init_node("tcp_skku_reciver")
        my_tcp = TCP_Server(host, port)

        # ros publish
        self.mission_pub = rospy.Publisher('/tcp_mission', Int8, queue_size=10)
        self.mission = Int8()
        self.lat_pub = rospy.Publisher('/tcp_lat', Float64, queue_size=10)
        self.lat = Float64()
        self.lon_pub = rospy.Publisher('/tcp_lon', Float64, queue_size=10)
        self.lon = Float64()
        self.heading_pub = rospy.Publisher('/tcp_heading', Float32, queue_size=10)
        self.heading = Float32()
        self.is_sim_pub = rospy.Publisher('/tcp_is_sim', Bool, queue_size=10)
        self.is_sim = Bool()

        r = rospy.Rate(1000)
        while not rospy.is_shutdown():
            try:
                # Subscribe
                self.mission.data, self.lat.data, self.lon.data, self.heading.data, self.is_sim.data = my_tcp.recv_data()
                self.mission_pub.publish(self.mission)
                self.lat_pub.publish(self.lat)
                self.lon_pub.publish(self.lon)
                self.heading_pub.publish(self.heading)
                self.is_sim_pub.publish(self.is_sim)
                r.sleep()
            except KeyboardInterrupt:
                try:
                    if self.my_tcp.conn:
                        self.my_tcp.conn.close()
                except:
                    pass
                break
        self.my_tcp.shutdown
        self.my_tcp.close()



if __name__ == '__main__':
    try:
        m = TcpManager()
    except rospy.ROSInterruptException:
        pass
