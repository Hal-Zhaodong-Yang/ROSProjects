#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class maze_runner_node:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometryCallback, queue_size = 1)
        self.obstacle_sub = rospy.Subscriber("/scan", LaserScan, self.lidarCallback, queue_size = 1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        self.Init = True
        self.Init_pos = Point()
        self.globalPos = Point()
        self.globalAng = 0
        self.Init_ang = 0
        self.mode = 0
        self.closestAngDegrees = 0
        self.closestDis = 10

    def odometryCallback(self, Odom):
        #rospy.loginfo(self.avoidingObstacle)
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

    def lidarCallback(self, ros_data):
        filteredLidarData = list(ros_data.ranges)
        for i in range(len(filteredLidarData)):
            if filteredLidarData[i] == 0:
                filteredLidarData[i] = 3.5
        self.closestDis = min(filteredLidarData)
        self.closestAngDegrees = filteredLidarData.index(self.closestDis)
        if self.mode == 0:
            self.getParallel()

    def getParallel(self):
        self.mode += 1
        turnAngle = self.closestAngDegrees
        howMuchToTurn = 0
        if turnAngle < 180:
            howMuchToTurn = turnAngle - 90
        else:
            howMuchToTurn = turnAngle - 270
        initialHeading = self.globalAng
        vel = Twist()
        initialTime = rospy.get_time()
        #while we are not within 5 degrees of desired location and it's been less than 36 seconds
        while abs(self.globalAng - (initialHeading + howMuchToTurn*math.pi/180)) > 5*math.pi/180  and rospy.get_time() - initialTime < 36.0:
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = math.pi/36 # 5 degrees per second --------------------------------------
            self.vel_pub.publish(vel)
        self.mode += 1
        vel.angular.z = 0
        self.vel_pub.publish(vel)
    

def main(args):
    ic = maze_runner_node()
    rospy.on_shutdown(ic.zeroSpeed)
    rospy.init_node('maze_runner_node', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)