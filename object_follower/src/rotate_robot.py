#!/usr/bin/env python

import rospy

from geometry_msgs.msgs import Point
from geometry_msgs.msgs import Twist

import sys

class rotate_robot:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/geometry_msgs/Point",
            Point, self.callback,  queue_size = 1)
        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        
    def callback(self, ros_data):
        posx = ros_data.x
        posw = ros_data.z

        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = (3.1416 / 4) * ((ros_data.x - 0.5 * ros_data.z) / (0.5 * ros_data.z))

        self.vel_pub.publish(vel)

def main(args):
    ic = rotate_robot()
    rospy.init_node('rotate_robot', anonymous = True)
    rospy.spin()


if __name__ == '__main__':
    # TODO
    main(sys.argv)