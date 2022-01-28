#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class goToGoal:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_Odometry, queue_size = 1)
        self.obstacle_sub = rospy.Subscriber("/scan", LaserScan, self.initObstacleAvoidance, queue_size = 1)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        self.Init = True
        self.Init_pos = Point()
        self.globalPos = Point()
        self.globalAng = 0
        self.Init_ang = 0

        self.goalPositions = [(1.5, 0), (1.5, 1.4), (0, 1.4)]
        self.mode = 0
        self.constantDistance = 0.2
        self.goalConstrain = 0.2
        self.maxSpeed = 0.1
        self.avoidingObstacle = False
        self.threshold = 3
        self.boxRange = []
        self.closestAng = 0
        self.closestDis = 10
        r = 0.2
        for i in range(20):
            self.boxRange.append(r / math.cos(i * 3 * math.pi / 180))
        for i in range(20, 40):
            self.boxRange.append(r * 2)
        for i in range(40, 61):
            self.boxRange.append(r / math.cos((180 - 3 * i) * math.pi / 180))



    def initObstacleAvoidance(self, ros_data):
        filteredLidarData = list(ros_data.ranges)
        for i in range(len(filteredLidarData)):
            if filteredLidarData[i] == 0:
                filteredLidarData[i] = 3.5
        self.closestDis = min(filteredLidarData)
        self.closestAng = filteredLidarData.index(self.closestDis)
        lidarBox = self.getLidarBox(filteredLidarData)
        badPointCounter = 0
        for i in range(61):
            if lidarBox[i] < self.boxRange[i]:
                badPointCounter += 1
        
        self.avoidingObstacle = badPointCounter > self.threshold

        




    def update_Odometry(self, Odom):

    
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

        vel = Twist()
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0

        obstacleAlert = False

        if self.avoidingObstacle:
            vel.angular.z = 2 * (1 - math.sin((self.closestAng + 0.05) * math.pi / 180)) * self.sign(-math.sin((self.closestAng + 0.05) * math.pi / 90))

        else:
            theta2 = self.getGoalAngle() - self.globalAng
            while theta2 < -math.pi:
                theta2 += 2 * math.pi
            while theta2 >= math.pi:
                theta2 -= 2 * math.pi
            if abs(theta2) > math.pi / 4:
                vel.angular.z = self.sign(theta2) * 2
            elif abs(theta2) > math.pi / 90:
                vel.angular.z = theta2 * 2.5 + self.sign(theta2) * math.pi / 90
            else:
                vel.angular.z = 0
            

        distance = math.sqrt((self.globalPos.x - self.goalPositions[self.mode][0]) ** 2 + (self.globalPos.y - self.goalPositions[self.mode][1]) ** 2)
        if self.avoidingObstacle:
            vel.linear.x = self.maxSpeed / 2
            self.vel_pub.publish(vel)
        elif distance >= self.goalConstrain:
            vel.linear.x = self.maxSpeed
            self.vel_pub.publish(vel)
        elif distance >= 0.03:
            vel.linear.x = distance * (self.maxSpeed - 0.02) / self.goalConstrain + 0.02
            self.vel_pub.publish(vel)
        else:
            vel.linear.x = 0
            vel.angular.z = 0
            self.vel_pub.publish(vel)
            self.pause()



    
    def pause(self):
        self.mode += 1
        while self.mode > 2:
            rospy.sleep(1)
        rospy.sleep(10)
    
    def getLidarBox(self, filteredLidar):
        theta1 = self.getGoalAngle()
        theta3 = theta1 + math.pi / 2 - self.globalAng
        while theta3 < 0:
            theta3 += 2 * math.pi
        while theta3 >= 2 * math.pi:
            theta3 -= 2 * math.pi
        theta_deg = int(theta3 * 180 / math.pi)
        lidarList = []
        for i in range(61):
            if theta_deg < 0:
                theta_deg += 360
            lidarList.append(filteredLidar[theta_deg])
            theta_deg -= 3

        return lidarList

    def getGoalAngle(self):
        return math.atan2(self.goalPositions[self.mode][1] - self.globalPos.y, self.goalPositions[self.mode][0] - self.globalPos.x)

    def sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

    def zeroSpeed(self):
        t = Twist()
        t.linear.x = 0
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0
        self.vel_pub.publish(t)




def main(args):
    ic = goToGoal()
    rospy.init_node('goToGoal', anonymous=True)
    rospy.on_shutdown(ic.zeroSpeed)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)