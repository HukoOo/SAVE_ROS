#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

#talktest
def test():
    rospy.init_node('talktest', anonymous=True)
    pub_cloud = rospy.Publisher("camera/depth_registered/points", PointCloud2)
    while not rospy.is_shutdown():
        pcloud = PointCloud2()
        # make point cloud
        cloud = [[33,22,11],[55,33,22],[33,22,11]]
        pcloud = pc2.create_cloud_xyz32(pcloud.header, cloud)
        pub_cloud.publish(pcloud)
        rospy.loginfo(pcloud)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass