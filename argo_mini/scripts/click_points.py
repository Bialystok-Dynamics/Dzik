#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped


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
    point2 = PointStamped()
    point2.header.stamp = rospy.Time.now()
    point2.header.frame_id = "/map"  # Zmień na swoją ramkę odniesienia
    point2.point.x = 2.0
    point2.point.y = 3.0
    point2.point.z = 4.0

    # Punkt 3,4,5 itd.

    points = [point1, point2]  # Dodaj więcej punktów do listy

    for point in points:
        rospy.loginfo("Publishing point at (%f, %f, %f)", point.point.x, point.point.y, point.point.z)
        pub.publish(point)
        rate.sleep()  # Opóźnienie między wysyłaniem punktów


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
