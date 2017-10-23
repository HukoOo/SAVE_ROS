#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket, time
import sys, copy
import rospy
from std_msgs.msg import Int8, Bool, Float64, Float32

host = '127.0.0.1'
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


class TCP_Server:
    def __init__(self, host, port):
        # self.mission_size = 1
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        address = (host, port)
        self.tcp_server.bind(address)
        self.tcp_server.listen(1)
        print 'Server listening....'
        self.conn, addr = self.tcp_server.accept()
        print 'Connected from...'
        print( addr)

    def recv_data(self):
        length_raw = self.conn.recv(4).decode("ascii")
        try:
            length = int(length_raw)
        except ValueError:
            length = 0
        # print length
        if length > 0:
            recv = self.conn.recv(length).decode("ascii")
            print recv
            recv_split = recv.split(",")
            return True
        else:
            # conn.close()
            # print " ----end else"
            return False


class TcpManager:
    def __init__(self):
        rospy.init_node("tcp_skku_reciver")
        self.my_tcp = TCP_Server(host, port)

        r = rospy.Rate(1000)
        while not rospy.is_shutdown():
            try:
                # Subscribe
                # self.my_tcp.recv_data()
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
