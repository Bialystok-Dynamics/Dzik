#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
import time


def talker():
    pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(2)  # 10hz
    # Punkt 1
    point1 = PointStamped()
    point1.header.stamp = rospy.Time.now()
    point1.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point1.point.x = 1.0
    point1.point.y = 2.0
    point1.point.z = 3.0

    # Punkt 2,3,4,5 itd.
    # point2 = PointStamped()
    # ...

    points = [point1]  # Dodaj więcej punktów do listy

    for point in points:
        rospy.loginfo("Publishing point at (%f, %f, %f)", point.point.x, point.point.y, point.point.z)
        pub.publish(point)
        time.sleep(1)  # Opóźnienie między wysyłaniem punktów
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
