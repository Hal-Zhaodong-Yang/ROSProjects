#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class getObjectRange:
    def __init__(self):
        self.constantDistance = 0.2
        self.LIDAR_sub = rospy.Subscriber("/scan", LaserScan, self.LIDAR_callback, queue_size = 1)
        self.obstacleLocation_pub = rospy.Publisher("/obstacleLocation", Point, queue_size = 1)
    
    def LIDAR_callback(self, ros_data):
        v = Point()
        v.x = min(ros_data.ranges)
        v.y = ros_data.ranges.index(v.x)
        if v.x <= self.constantDistance:
            self.obstacleLocation_pub.publish(v)
        

def main(args):
    ic = getObjectRange()
    rospy.init_node('getObjectRange', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)