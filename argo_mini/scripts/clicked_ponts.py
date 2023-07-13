#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
import time


def talker():
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(2)

    point1 = PointStamped()
    point1.header.stamp = rospy.Time.now()
    point1.header.seq = 0
    point1.header.frame_id = "/map"
    point1.point.x = 7.
    point1.point.y = 11.
    point1.point.z = 0.

    point2 = PointStamped()
    point2.header.stamp = rospy.Time.now()
    point2.header.seq = 1
    point2.header.frame_id = "/map"
    point2.point.x = 7.
    point2.point.y = -11.
    point2.point.z = 0.

    point3 = PointStamped()
    point3.header.stamp = rospy.Time.now()
    point3.header.seq = 2
    point3.header.frame_id = "/map"
    point3.point.x = -7.
    point3.point.y = -11.
    point3.point.z = 0.

    point4 = PointStamped()
    point4.header.stamp = rospy.Time.now()
    point4.header.seq = 3
    point4.header.frame_id = "/map"
    point4.point.x = -7.
    point4.point.y = 11.
    point4.point.z = 0.

    point5 = PointStamped()
    point5.header.stamp = rospy.Time.now()
    point2.header.seq = 4
    point5.header.frame_id = "/map"
    point5.point.x = -2.
    point5.point.y = -1.
    point5.point.z = 0.

    points = [point1, point2, point3, point4, point5]
    # rospy.loginfo(f"Publishing point at {points}")
                  
    for point in points:
            rate.sleep()
            rospy.loginfo("Publishing point at (%f, %f, %f)", point.point.x, point.point.y, point.point.z)
            pub.publish(point)
            

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
