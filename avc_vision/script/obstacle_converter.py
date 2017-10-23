#!/usr/bin/env python
import rospy, math
import time
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import PolygonStamped, Point32
from teb_local_planner.msg import ObstacleMsg, SKKU_LaneArray, SKKU_Lane, SKKU_CenterLaneArray
# lidar
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
# detected object
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class Obstacles:
    def __init__(self):
        # Subscriber
        # self.scale_sub = rospy.Subscriber('/global_path/scale', Float32, self.CB_scale)
        self.scale_factor = 1.0
 
        self.lane_sub = rospy.Subscriber('/lane_array', SKKU_LaneArray, self.CB_lane)
        self.lane_array = SKKU_LaneArray()
        self.lane_array.lane = [SKKU_Lane(), SKKU_Lane()]
        self.lane_array.lane[0].left = 10
        self.lane_array.lane[0].right = -10
        self.lane_array.lane[1].left = 10
        self.lane_array.lane[1].right = -10

        self.raw = True
        if self.raw: self.lidar_sub = rospy.Subscriber('/Sensor/clustered_points', PointCloud2, self.CB_lidar)
        else: self.obj_sub = rospy.Subscriber('/Sensor/visualization_objects', MarkerArray, self.CB_obj)
        self.lidar_array = []
        self.lidar_array_used = True # Semaphore

        self.raw_tracking_sub = rospy.Subscriber('/Sensor/filtered_objects', MarkerArray, self.CB_raw_tracking)

        self.mission_sub = rospy.Subscriber('/cRio_mission', Int8, self.CB_mission)
        self.mission = 1

        # Publisher
        self.obs_pub = rospy.Publisher('/obstacle_converter/obstacles', ObstacleMsg, queue_size=100) # already * scale
        self.tracking_target_pub = rospy.Publisher('/obstacle_converter/tracking_target', Point32, queue_size=10)
        self.tracking_target_info = Point32()
        self.center_lane_pub = rospy.Publisher('/obstacle_converter/center_lane', SKKU_CenterLaneArray, queue_size=20) # std m scaleobstacle_msg
        self.center_lane = SKKU_CenterLaneArray()
        self.center_lane.center = [0.0, 0.0, 0.0, 0.0, 0.0]

    # def CB_scale(self, data):
    #     self.scale_factor = data.data

    def CB_lane(self, data):
        # 3
        self.lane_array.lane[0].left = -data.lane[0].left
        self.lane_array.lane[0].right = -data.lane[0].right
        # 15
        self.lane_array.lane[1].left = -data.lane[1].left
        self.lane_array.lane[1].right = -data.lane[1].right        
        # center: 3, 6, 9, 12, 15
        mid_a = (-data.lane[0].left - data.lane[0].right)/10.0
        mid_b = (-data.lane[1].left - data.lane[1].right)/10.0
        for i in xrange(5):
            self.center_lane.center[i] = (mid_a*(5-i)) + (mid_b*i)

    def CB_lidar(self, data):
        height = data.height / 2.0
        middle_x = data.width / 2.0
        if (height >= data.height) or (middle_x >= data.width): return -1
        if self.lidar_array_used:
            self.lidar_array = []
            for point in pc2.read_points(data, skip_nans=True):
                self.lidar_array.append((point[0], point[1]))
            self.lidar_array_used = False

    def CB_obj(self, marker_array):
        if self.lidar_array_used:
            self.lidar_array = []
            for i in xrange(len(marker_array.markers)):
                max_x, max_y = marker_array.markers[i].points[0].x, marker_array.markers[i].points[0].y
                min_x, min_y = marker_array.markers[i].points[1].x, marker_array.markers[i].points[1].y
            #for marker in marker_array.markers:
            #    max_x, max_y = marker.points[0].x, marker.points[0].y
            #    min_x, min_y = marker.points[1].x, marker.points[1].y
                self.lidar_array.append([(max_x, max_y), (max_x, min_y), (min_x, min_y), (min_x, max_y)])
            self.lidar_array_used = False

    def CB_raw_tracking(self, roi_marker_array):
        min_x, center_y = 999.99, 999.99
        at_least = False
        for i in xrange(len(roi_marker_array.markers)):
            if (min_x > roi_marker_array.markers[i].points[1].x) and (roi_marker_array.markers[i].points[1].x > 0):
                min_x = roi_marker_array.markers[i].points[1].x
                center_y = (roi_marker_array.markers[i].points[0].y + roi_marker_array.markers[i].points[1].y)/2.0
                at_least = True

        if at_least:

            #rospy.loginfo(len(roi_marker_array.markers))
            #rospy.loginfo("[%f,%f][%f,%f]", roi_marker_array.markers[0].points[0].x, roi_marker_array.markers[0].points[0].y, roi_marker_array.markers[0].points[1].x, roi_marker_array.markers[0].points[1].y)

            self.tracking_target_info.x = min_x
            self.tracking_target_info.y = center_y
            self.tracking_target_info.z = math.sqrt(pow(min_x,2)+pow(center_y,2))
            rospy.loginfo("x: %f, y: %f", self.tracking_target_info.x, self.tracking_target_info.y)

    def CB_mission(self, data):
        self.mission = data.data

    # def add_obstacle(self, idx, x, y):
    #     try:
    #         self.obstacle_msg.obstacles[idx].polygon.points[0].x = x
    #         self.obstacle_msg.obstacles[idx].polygon.points[0].y = y
    #     except IndexError:
    #         self.obstacle_msg.obstacles.append(PolygonStamped())
    #         self.obstacle_msg.obstacles[idx].polygon.points = [Point32()]
    #         self.obstacle_msg.obstacles[idx].polygon.points[0].x = x
    #         self.obstacle_msg.obstacles[idx].polygon.points[0].y = y


    def publish(self):
        self.center_lane_pub.publish(self.center_lane)
        self.tracking_target_pub.publish(self.tracking_target_info)
        # obstacles: Local variable
        obs_msg = ObstacleMsg()
        obs_msg.header.frame_id = "odom"
        obs_msg.header.stamp = rospy.Time.now()
        k = 0

        if (self.mission == 1):
            # lane
            obs_msg.obstacles.append(PolygonStamped())
            obs_msg.obstacles[k].polygon.points = [Point32(), Point32()]
            obs_msg.obstacles[k].polygon.points[0].x = 3.0 * self.scale_factor
            obs_msg.obstacles[k].polygon.points[0].y = self.lane_array.lane[0].left * self.scale_factor
            obs_msg.obstacles[k].polygon.points[1].x = 15.0 * self.scale_factor
            obs_msg.obstacles[k].polygon.points[1].y = self.lane_array.lane[1].left * self.scale_factor
            k += 1
            obs_msg.obstacles.append(PolygonStamped())
            obs_msg.obstacles[k].polygon.points = [Point32(), Point32()]
            obs_msg.obstacles[k].polygon.points[0].x = 3.0 * self.scale_factor
            obs_msg.obstacles[k].polygon.points[0].y = self.lane_array.lane[0].right * self.scale_factor
            obs_msg.obstacles[k].polygon.points[1].x = 15.0 * self.scale_factor
            obs_msg.obstacles[k].polygon.points[1].y = self.lane_array.lane[1].right * self.scale_factor
            k += 1

        if (self.mission != 3):
            # lidar
            lidar_len = len(self.lidar_array)
            for i in xrange(lidar_len):
                obs_msg.obstacles.append(PolygonStamped())
                if self.raw:
                    obs_msg.obstacles[k].polygon.points = [Point32()]
                    obs_msg.obstacles[k].polygon.points[0].x = self.lidar_array[i][0] * self.scale_factor
                    obs_msg.obstacles[k].polygon.points[0].y = self.lidar_array[i][1] * self.scale_factor
                else:
                    v0 = Point32()
                    v1 = Point32()
                    v2 = Point32()
                    v3 = Point32()
                    try:
                        v0.x, v0.y = self.lidar_array[i][0]
                        v1.x, v1.y = self.lidar_array[i][1]
                        v2.x, v2.y = self.lidar_array[i][2]
                        v3.x, v3.y = self.lidar_array[i][3]
                    except IndexError:
                        rospy.loginfo("*******************i %d", i)

                    v0.x, v0.y = v0.x*self.scale_factor, v0.y*self.scale_factor
                    v1.x, v1.y = v1.x*self.scale_factor, v1.y*self.scale_factor
                    v2.x, v2.y = v2.x*self.scale_factor, v2.y*self.scale_factor
                    v3.x, v3.y = v3.x*self.scale_factor, v3.y*self.scale_factor
                    try:
                        obs_msg.obstacles[k].polygon.points = [v0, v1, v2, v3]
                    except IndexError:
                        rospy.loginfo("*******************k %d", k)
                k += 1

        self.obs_pub.publish(obs_msg)
        #rospy.loginfo("----------------------------")
        #rospy.loginfo(obs_msg.header.frame_id)
        #rospy.loginfo(obs_msg.header.stamp)
        #i = 0
        #while True:
        #    try:
        #        rospy.loginfo("idx: %d  x: %.1f, y: %.1f", i, obs_msg.obstacles[i].polygon.points[0].x , obs_msg.obstacles[i].polygon.points[0].y)
        #    except IndexError:
        #        break
        #    i += 1000
        

# 40m
if __name__ == '__main__':
    try:
        rospy.init_node("obstacle_converter")
        obs = Obstacles()

        r = rospy.Rate(40)
        while not rospy.is_shutdown():
            if not obs.lidar_array_used:
                obs.publish()
                obs.lidar_array_used = True
            r.sleep()
    except rospy.ROSInterruptException:
        pass





#_______________________________________________________________
# # Add point obstacle
# obstacle_msg.obstacles.append(PolygonStamped())
# obstacle_msg.obstacles[0].polygon.points = [Point32()]
# obstacle_msg.obstacles[0].polygon.points[0].x = 1.5
# obstacle_msg.obstacles[0].polygon.points[0].y = 0
# obstacle_msg.obstacles[0].polygon.points[0].z = 0


# # Add line obstacle
# obstacle_msg.obstacles.append(PolygonStamped())
# line_start = Point32()
# line_start.x = -2.5
# line_start.y = 0.5
# #line_start.y = -3
# line_end = Point32()
# line_end.x = -2.5
# line_end.y = 2
# #line_end.y = -4
# obstacle_msg.obstacles[1].polygon.points = [line_start, line_end]

# # Add polygon obstacle
# obstacle_msg.obstacles.append(PolygonStamped())
# v1 = Point32()
# v1.x = -1
# v1.y = -1
# v2 = Point32()
# v2.x = -0.5
# v2.y = -1.5
# v3 = Point32()
# v3.x = 0
# v3.y = -1
# obstacle_msg.obstacles[2].polygon.points = [v1, v2, v3]

