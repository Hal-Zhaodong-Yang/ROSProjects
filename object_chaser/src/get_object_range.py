#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class get_object_range:
    def __init__(self):
        self.angle = -1
        self.depth = -1
        self.angle_sub = rospy.Subscriber("/geometry_msgs/Point", Point, self.angle_callback, queue_size = 1)
        self.LIDAR_sub = rospy.Subscriber("/scan", LaserScan, self.LIDAR_callback, queue_size = 1)
        self.location_pub = rospy.Publisher("/geometry_msgs/Location", Point, queue_size = 1)

    def angle_callback(self, ros_data):
        self.angle = ros_data.z
        if self.depth < 0:
            break
        if ros_data.z < 0:
            self.angle = ros_data.z
            break
        #self.angle = ros_data.z
        newLocation = Point()
        newLocation.x = self.angle
        newLocation.y = self.depth
        self.location_pub.publish(newLocation)
    
    def LIDAR_callback(self, ros_data):
        if self.angle < 0:
            break
        index = int(self.angle/(3.1415926) * 180)
        self.depth = ros_data.ranges[index]
        

def main(args):
    ic = get_object_range()
    rospy.init_node('get_object_range', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)