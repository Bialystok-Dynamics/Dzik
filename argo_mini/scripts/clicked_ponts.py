#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
import time


def talker():
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 10hz

    # Punkt 1
    rate.sleep()
    point1 = PointStamped()
    point1.header.stamp = rospy.Time.now()
    point1.header.seq = 0
    point1.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point1.point.x = 7.
    point1.point.y = 11.
    point1.point.z = 0.

    # Punkt 2,3,4,5 itd.
    # point2 = PointStamped()
    # ...
    # Punkt 2
    rate.sleep()
    point2 = PointStamped()
    point2.header.stamp = rospy.Time.now()
    point2.header.seq = 1
    point2.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point2.point.x = 7.
    point2.point.y = -11.
    point2.point.z = 0.

    # Punkt 3
    rate.sleep()
    point3 = PointStamped()
    point3.header.stamp = rospy.Time.now()
    point3.header.seq = 2
    point3.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point3.point.x = -7.
    point3.point.y = -11.
    point3.point.z = 0.

     # Punkt 4
    rate.sleep()
    point4 = PointStamped()
    point4.header.stamp = rospy.Time.now()
    point4.header.seq = 3
    point4.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point4.point.x = -7.
    point4.point.y = 11.
    point4.point.z = 0.

     # Punkt 5
    rate.sleep()
    point5 = PointStamped()
    point5.header.stamp = rospy.Time.now()
    point2.header.seq = 4
    point5.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point5.point.x = 1.427663803100586
    point5.point.y = -0.9947977066040039
    point5.point.z = 0.0007171630859375

    points = [point2, point1, point3, point4, point5]  # Dodaj więcej punktów do listy
    rospy.loginfo(f"Publishing point at {points}")
                  
    for point in points:
            rate.sleep()
            #rozpoczecie odmierzania czasu
        # while not rospy.is_shutdown():
            rospy.loginfo("Publishing point at (%f, %f, %f)", point.point.x, point.point.y, point.point.z)
            pub.publish(point)
            


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass