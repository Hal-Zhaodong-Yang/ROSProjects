#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class chase_object:
    def __init__(self):
        self.img_sub = rospy.Subscriber("/geometry_msgs/Location", Point, self.callback, queue_size = 1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        self.e_angle = []
        self.e_depth = []


    def callback(self, ros_data):
        e_angle.append(ros_data.x if (ros_data.x < 3.1415926535897) else ros_data.x - 3.1415926535897 * 2)
        e_depth.append(ros_data.y - 0.2)
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = -0.75 * (3.1416/4)*((ros_data.x - 0.5*ros_data.z) / (0.5*ros_data.z))
        rospy.loginfo("vel_info:")
        rospy.loginfo(vel.angular.z)
        self.vel_pub.publish(vel)

def main(args):
    ic = chase_object()
    rospy.init_node('chase_object', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)