#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
import numpy as np
import time
from std_srvs.srv import Empty


class PointPublisher:
    
    def __init__(self):
        self.pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
        
        self.sub = rospy.Subscriber('/argo_mini/centroids', Marker, self.centroids_callback)
        self.rate = rospy.Rate(2)
        self.last_point_published_time = None
        self.centroids_empty_time = None
        self.start_time = rospy.Time.now()
        self.flag = True
        self.last_non_empty_centroid_time = None
        self.centroids_logged = False
        self.msub = rospy.Subscriber('/argo_mini/cmd_vel', Twist,self.cmd_callback)
        self.mapsub = rospy.Subscriber('/argo_mini/map', OccupancyGrid, self.subscriber)
        self.timeout_flag = False
        # czas jaki uplynal od ostatniego komunikatu cmd_vel
        self.ros_vel_time = rospy.Time.now()

        self.map_array = []


    def subscriber(self, data):
        self.map_array = np.array(data.data, dtype=np.int8)

    def cmd_callback(self, data):
        self.ros_vel_time = rospy.Time.now()
        self.timeout_flag = False

    def centroids_callback(self, data):
        # Jeżeli marker nie jest pusty, zaktualizuj czas ostatniego niepustego markeru
        if data.points:
            self.last_non_empty_centroid_time = rospy.Time.now()
            self.centroids_logged = False  # Resetuj flagę po otrzymaniu niepustego markeru

         # Jeżeli marker jest pusty, sprawdź czy minęło 15 sekund od ostatniego niepustego markeru
        elif self.timeout_flag == True and self.last_non_empty_centroid_time and (rospy.Time.now() - self.last_non_empty_centroid_time).to_sec() > 15.0:
            if not self.centroids_logged and (rospy.Time.now() - self.start_time).to_sec() > 30.0 and self.flag == True:
                self.centroids_empty_time = rospy.Time.now()
                if self.last_point_published_time:
                    self.flag = False
                    diff = self.centroids_empty_time - self.last_point_published_time  # Odejmij 15 sekund
                    rospy.loginfo("Czas trwania mapowania [s]: %f", diff.to_sec())
                    self.centroids_logged = True  # Ustaw flagę, że komunikat został zalogowany
                    #zapis mapy
                    zero_count = np.count_nonzero(self.map_array == 0)
                    one_count = np.count_nonzero(self.map_array == 100)
                    minusone_count = np.count_nonzero(self.map_array == -1)
                    print("Liczba wolnych punktow w mapie zajętości: ", zero_count)
                    print("Liczba zajetych punktow w mapie zajętości: ", one_count)
                    print("Liczba nieznanych punktow w mapie zajętości: ", minusone_count)

                    # rospy.wait_for_service('map_save_print')
                    # try:
                    #     empty_service_proxy = rospy.ServiceProxy('map_save_print', Empty)
                    #     response = empty_service_proxy()
                    #     return response
                    # except rospy.ServiceException as e:
                    #     print("Service call failed: %s"%e)
    def talker(self):
        points = self.prepare_points()

        for point in points:
            self.rate.sleep()
            rospy.loginfo("Publishing point at (%f, %f, %f)", point.point.x, point.point.y, point.point.z)
            self.pub.publish(point)
            self.last_point_published_time = rospy.Time.now()
            self.vel_last_point_published_time = rospy.Time.now()



    @staticmethod
    def prepare_points():
        points = []

        point1 = PointStamped()
        point1.header.stamp = rospy.Time.now()
        point1.header.seq = 0
        point1.header.frame_id = "/map"
        point1.point.x = 17.
        point1.point.y = 19.
        point1.point.z = 0.

        point2 = PointStamped()
        point2.header.stamp = rospy.Time.now()
        point2.header.seq = 1
        point2.header.frame_id = "/map"
        point2.point.x = 17.
        point2.point.y = -19.
        point2.point.z = 0.

        point3 = PointStamped()
        point3.header.stamp = rospy.Time.now()
        point3.header.seq = 2
        point3.header.frame_id = "/map"
        point3.point.x = -15.
        point3.point.y = -19.
        point3.point.z = 0.

        point4 = PointStamped()
        point4.header.stamp = rospy.Time.now()
        point4.header.seq = 3
        point4.header.frame_id = "/map"
        point4.point.x = -15.
        point4.point.y = 19.
        point4.point.z = 0.

        point5 = PointStamped()
        point5.header.stamp = rospy.Time.now()
        point2.header.seq = 4
        point5.header.frame_id = "/map"
        point5.point.x = -1.
        point5.point.y = -1.
        point5.point.z = 0.

        points = [point4, point3, point2, point1, point5]
        
        return points
    


if __name__ == '__main__':
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(2)
    point_publisher = PointPublisher()   
    try:
        point_publisher.talker()
        while not rospy.is_shutdown():
            rate.sleep()
            if (rospy.Time.now() - point_publisher.ros_vel_time).to_sec() > 10.0:
                rospy.loginfo("STOI")
                point_publisher.timeout_flag = True
    except rospy.ROSInterruptException:
        pass
