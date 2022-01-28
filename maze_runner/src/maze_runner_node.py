#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class maze_node:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odometryCallback, queue_size = 1)
        self.obstacle_sub = rospy.Subscriber("/scan", LaserScan, self.lidarCallback, queue_size = 1)
        self.image_sub = rospy.Subscriber("/image_sign", Point, self.imageCallback, queue_size = 2)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)
        self.Init = True
        self.odometryComplete = False
        self.Init_pos = Point()
        self.globalPos = Point()
        self.globalAng = 0
        self.Init_ang = 0
        self.mode = 3
        self.closestAngDegrees = 0
        self.closestOverallDegrees = 0
        self.closestOverallDis = 10
        self.closestDis = 10
        # used to identify shape in image (mode 0)
        self.imageList = []
        self.imageListLength = 50
        self.imageListSelections = 10
        # used during recovery mode (mode 0.5)
        self.recoveryImage = -1
        self.reverseVelocity = -0.03
        # used to turn certain multiple of 90 degrees (mode 1)
        self.turnDirection = 0
        self.initialDesiredAngleRad = 0
        # used to go straight and stop (mode 2)
        self.wallDistance = 0.47
        self.forwardVelocity = 0.08
        self.rangeOfViewForStopping = 13 # Check from 13 degrees to -13 degrees for a wall
        self.maxLidarTooCloseCounter = 5 # If 5 points are less than stopping distance, robot stops before wall
        self.goStraightAng = 0
        # used to align (mode 3)

    def odometryCallback(self, Odom):
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
        self.globalAng = self.wrap(self.globalAng)
        self.odometryComplete = True

    def lidarCallback(self, ros_data):
        filteredLidarData = list(ros_data.ranges)
        for i in range(len(filteredLidarData)):
            if filteredLidarData[i] == 0:
                filteredLidarData[i] = 3.5
        self.closestOverallDis = min(filteredLidarData)
        self.closestOverallDegrees = filteredLidarData.index(self.closestOverallDis)
        #350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
        frontViews = filteredLidarData[-self.rangeOfViewForStopping:] + filteredLidarData[0:self.rangeOfViewForStopping]
        leftViews = filteredLidarData[75:105]
        rightViews = filteredLidarData[255:285]
        self.closestDis = min(frontViews)
        self.closestAngDegrees = frontViews.index(self.closestDis)
        self.closestAngDegrees -= self.rangeOfViewForStopping
        if not self.odometryComplete:
            return 0
        elif self.mode == 1:
            vel = Twist()
            if abs(self.globalAng - self.initialDesiredAngleRad) > 1*math.pi/180:
                #rospy.loginfo("TURNING!!!")
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = self.turnDirection*math.pi/27 # 7.5 degrees per second
            else:
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = 0
                self.mode = 2
                rospy.loginfo("STATE REACHED 2!")
            self.vel_pub.publish(vel)
        elif self.mode == 2:
            self.goStraight(frontViews, leftViews, rightViews)
        elif self.mode == 3:
            vel = Twist()
            #rospy.loginfo("ALIGN?????????????")
            rospy.loginfo("frontview closest Ang")
            rospy.loginfo(self.closestAngDegrees)
            if self.closestAngDegrees != 0:
                rospy.loginfo("ALIGNING!!!")
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = self.sign(self.closestAngDegrees)*math.pi/90 # 2 degrees per second
            else:
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = 0
                self.mode = 0
                rospy.loginfo("STATE REACHED 0!")
            self.vel_pub.publish(vel)
    
    def imageCallback(self, ros_data):
        if not self.odometryComplete:
            return 0
        if self.mode == 2:
            if ros_data.x > 0 and ros_data.y < 0.75 and ros_data.y > 0.25:
                self.goStraightAng = 0.5 - ros_data.y
            else:
                self.goStraightAng = 0
        if self.recoveryImage != -1 and self.mode == 0:
            rospy.loginfo("We have a recovery image")
            if self.recoveryImage == 1:
                self.initialDesiredAngleRad = self.globalAng + 90*math.pi/180
                self.turnDirection = 1
            elif self.recoveryImage == 2:
                self.initialDesiredAngleRad = self.globalAng - 90*math.pi/180
                self.turnDirection = -1
            elif self.recoveryImage == 3 or self.recoveryImage == 4:
                self.initialDesiredAngleRad = self.globalAng + 180*math.pi/180
                self.turnDirection = 1
            elif self.recoveryImage == 5:
                self.mode = 4
                rospy.loginfo("SOLVED THE MAZE!! :)))))")
                return 0
            rospy.loginfo("Retrieving image recovery value:")
            rospy.loginfo(self.recoveryImage)
            self.initialDesiredAngleRad = self.wrap(self.initialDesiredAngleRad)
            rospy.loginfo("Initial desired angle:")
            rospy.loginfo(self.initialDesiredAngleRad)
            self.recoveryImage = -1
            self.mode = 1
            rospy.loginfo("STATE REACHED 1!")

        if self.mode == 0.5:
            rospy.loginfo("Still in recovery mode...")
            if len(self.imageList) < self.imageListLength:
                rospy.loginfo("TAKING PICS!!!")
                self.imageList.append(ros_data.x)
            else:
                lastImages = self.imageList[-1*self.imageListSelections:-1]
                counter = 0
                pictureValue = lastImages[0]
                for i in lastImages:
                    curr_frequency = lastImages.count(i)
                    if (curr_frequency > counter):
                        counter = curr_frequency
                        pictureValue = i
                if pictureValue == 5:
                    self.mode = 4
                    vel = Twist()
                    vel.linear.x = 0
                    vel.linear.y = 0
                    vel.linear.z = 0
                    vel.angular.x = 0
                    vel.angular.y = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    rospy.loginfo("SOLVED THE MAZE!! :)))))")
                    return 0
                elif pictureValue > 0:
                    self.recoveryImage = pictureValue
                    vel = Twist()
                    vel.linear.x = 0
                    vel.linear.y = 0
                    vel.linear.z = 0
                    vel.angular.x = 0
                    vel.angular.y = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    self.mode = 2
                    rospy.loginfo("FOUND A RECOVERY IMAGE. EXITING RECOVERY MODE!")
                    self.imageListLength = 50
                else:
                    self.imageList = []
            
        if self.mode == 0:
            if len(self.imageList) < self.imageListLength:
                #rospy.loginfo("TAKING PICS!!!")
                self.imageList.append(ros_data.x)
            else:
                lastImages = self.imageList[-1*self.imageListSelections:-1]
                counter = 0
                pictureValue = lastImages[0]
                for i in lastImages:
                    curr_frequency = lastImages.count(i)
                    if (curr_frequency > counter):
                        counter = curr_frequency
                        pictureValue = i
                if pictureValue == 0:
                    self.mode = 0.5
                    rospy.loginfo("STATE REACHED 0.5! ENTERING RECOVERY MODE")
                    self.imageList = []
                    self.imageListLength = 10 # speeds up the time it takes to accumulate pictures
                    vel = Twist()
                    vel.linear.x = self.reverseVelocity
                    vel.linear.y = 0
                    vel.linear.z = 0
                    vel.angular.x = 0
                    vel.angular.y = 0
                    vel.angular.z = 0
                    self.vel_pub.publish(vel)
                    return 0
                elif pictureValue == 1:
                    self.initialDesiredAngleRad = self.globalAng + 90*math.pi/180
                    self.turnDirection = 1
                elif pictureValue == 2:
                    self.initialDesiredAngleRad = self.globalAng - 90*math.pi/180
                    self.turnDirection = -1
                elif pictureValue == 3 or pictureValue == 4:
                    self.initialDesiredAngleRad = self.globalAng + 180*math.pi/180
                    self.turnDirection = 1
                elif pictureValue == 5:
                    self.mode = 4
                    rospy.loginfo("SOLVED THE MAZE!! :)))))")
                    return 0
                rospy.loginfo("Identified picture value:")
                rospy.loginfo(pictureValue)
                self.initialDesiredAngleRad = self.wrap(self.initialDesiredAngleRad)
                rospy.loginfo("Initial desired angle:")
                rospy.loginfo(self.initialDesiredAngleRad)
                self.mode = 1
                rospy.loginfo("STATE REACHED 1!")

    def goStraight(self, frontViews, leftViews, rightViews):
        tooClose = 0
        for i in range(0, len(frontViews)):
            if frontViews[i] < self.wallDistance:
                tooClose += 1
        if tooClose > self.maxLidarTooCloseCounter:
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            self.vel_pub.publish(vel)
            self.mode = 3
            rospy.loginfo("STATE REACHED 3!")
            self.imageList = []
            return 0
        leftViews = np.array(leftViews)
        rightViews = np.array(rightViews)
        left_previous = leftViews[:-1]
        left_after = leftViews[1:]
        right_previous = rightViews[:-1]
        right_after= rightViews[1:]
        max_left = np.max(leftViews)
        max_right = np.max(rightViews)
        max_left_diff = np.max(np.abs(left_previous - left_after))
        max_right_diff = np.max(np.abs(right_previous - right_after))
        haveALeftWall = False
        haveARightWall = False
        if max_left < 0.6 and max_left_diff < 0.05:
            haveALeftWall = True
        if max_right < 0.6 and max_right_diff < 0.05:
            haveARightWall = True
        vel = Twist()
        vel.linear.x = self.forwardVelocity
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        # if we have a wall to follow
        #rospy.loginfo("leftbool")
        #rospy.loginfo(haveALeftWall)
        #rospy.loginfo("rightbool")
        #rospy.loginfo(haveARightWall)
        #rospy.loginfo("threshold info")
        #rospy.loginfo(max_left)
        #rospy.loginfo(max_right)
        #rospy.loginfo(max_left_diff)
        #rospy.loginfo(max_right_diff)
        #rospy.loginfo("closestAngDegree")
        #rospy.loginfo(self.closestAngDegrees)

        if (haveALeftWall and (self.closestOverallDegrees > 75 and self.closestOverallDegrees < 105)) or (haveARightWall and (self.closestOverallDegrees > 225 and self.closestOverallDegrees < 285)):
            vel.angular.z = 0.04 * (1 - math.sin((self.closestOverallDegrees + 0.05) * math.pi / 180)) * self.sign(-math.sin((self.closestOverallDegrees + 0.05) * math.pi / 90))
            if haveALeftWall:
                rospy.loginfo("Following left wall")
            else:
                rospy.loginfo("Following right wall")
        else:
            vel.angular.z = 0
        if self.goStraightAng != 0:
            rospy.loginfo("Chasing the picture")
            vel.angular.z = self.goStraightAng
            #vel.angular.z = 0
        rospy.loginfo("-----------------------------------------------------------------------")
        self.vel_pub.publish(vel)

    def sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0
    
    def wrap(self, angle):
        if angle > math.pi:
            angle = angle - 2*math.pi
        elif angle < -math.pi:
            angle = angle + 2*math.pi
        return angle

def main(args):
    flag = 1
    rospy.init_node('maze_runner_node', anonymous=True)
    if flag == 1:
        ic = maze_node()
        flag = 0
    rospy.loginfo("ros spin")
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)