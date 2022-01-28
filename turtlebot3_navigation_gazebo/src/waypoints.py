#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
import actionlib

class waypoints:
    def __init__(self):
        #self.odom_sub = rospy.Subscriber("/odom", Odometry, self.update_Odometry, queue_size = 1)
        #self.obstacle_sub = rospy.Subscriber("/scan", LaserScan, self.initObstacleAvoidance, queue_size = 1)
        #self.waypoint_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        #self.waypoint_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback, queue_size = 1)
        #f = open("/home/hal/catkin7785/src/turtlebot3_simulations/turtlebot3_gazebo/src/global_waypoints.txt")
        #f.close()

        self.mode = 0
        #self.flag = 0
        #self.init = 0
        self.last_text = "initial"
        self.last_status = 0




        #sim

        newPoseStamped1 = PoseStamped()
        newPoseStamped1.header.frame_id = "map"
        newPoseStamped1.pose.position.x = 2.57
        newPoseStamped1.pose.position.y = 0.53
        newPoseStamped1.pose.position.z = 0.000
        newPoseStamped1.pose.orientation.x = 0.000
        newPoseStamped1.pose.orientation.y = 0.000
        newPoseStamped1.pose.orientation.z = 0.96
        newPoseStamped1.pose.orientation.w = -0.256

        newPoseStamped2 = PoseStamped()
        newPoseStamped2.header.frame_id = "map"
        newPoseStamped2.pose.position.x = 3.45
        newPoseStamped2.pose.position.y = -0.6
        newPoseStamped2.pose.position.z = 0.000
        newPoseStamped2.pose.orientation.x = 0.000
        newPoseStamped2.pose.orientation.y = 0.000
        newPoseStamped2.pose.orientation.z = 0.9999
        newPoseStamped2.pose.orientation.w = 0.008

        newPoseStamped3 = PoseStamped()
        newPoseStamped3.header.frame_id = "map"
        newPoseStamped3.pose.position.x = 0.345
        newPoseStamped3.pose.position.y = -0.56
        newPoseStamped3.pose.position.z = 0.000
        newPoseStamped3.pose.orientation.x = 0.000
        newPoseStamped3.pose.orientation.y = 0.000
        newPoseStamped3.pose.orientation.z = 0.0
        newPoseStamped3.pose.orientation.w = 1.0
        






        #real
        '''
        newPoseStamped1 = PoseStamped()
        newPoseStamped1.header.frame_id = "map"
        newPoseStamped1.pose.position.x = -2.14
        newPoseStamped1.pose.position.y = 4.0
        newPoseStamped1.pose.position.z = 0.000
        newPoseStamped1.pose.orientation.x = 0.000
        newPoseStamped1.pose.orientation.y = 0.000
        newPoseStamped1.pose.orientation.z = 0.96
        newPoseStamped1.pose.orientation.w = -0.256

        newPoseStamped2 = PoseStamped()
        newPoseStamped2.header.frame_id = "map"
        newPoseStamped2.pose.position.x = -0.91
        newPoseStamped2.pose.position.y = 4.43
        newPoseStamped2.pose.position.z = 0.000
        newPoseStamped2.pose.orientation.x = 0.000
        newPoseStamped2.pose.orientation.y = 0.000
        newPoseStamped2.pose.orientation.z = 0.9999
        newPoseStamped2.pose.orientation.w = 0.008

        newPoseStamped3 = PoseStamped()
        newPoseStamped3.header.frame_id = "map"
        newPoseStamped3.pose.position.x = -1.82
        newPoseStamped3.pose.position.y = 2.09
        newPoseStamped3.pose.position.z = 0.000
        newPoseStamped3.pose.orientation.x = 0.000
        newPoseStamped3.pose.orientation.y = 0.000
        newPoseStamped3.pose.orientation.z = 0.0
        newPoseStamped3.pose.orientation.w = 1.0
        '''


        









        self.poses = [newPoseStamped1, newPoseStamped2, newPoseStamped3]

    def movebase_client(self):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = self.poses[self.mode]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

    def incrementMode(self):
        self.mode += 1
        rospy.loginfo("Mode update: ")
        rospy.loginfo(self.mode)

    def pause(self):
        rospy.loginfo(self.mode)
        rospy.sleep(2)
        while self.mode > 2:
            rospy.sleep(10)

def main(args):
    ic = waypoints()
    flag = 0
    rospy.init_node('waypoints', anonymous=True)
    if not flag:
        flag = 1
    for i in range(0, 3):
        result = ic.movebase_client()
        if result:
            rospy.loginfo("Sucess! Pausing...")
            ic.pause()
            ic.incrementMode()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)



'''
        
    def navigate(self):
        self.waypoint_pub.publish(self.poses[self.mode])
        #self.waypoint_pub.publish(self.poses[self.mode])
        #self.waypoint_pub.publish(self.poses[self.mode])
        #self.pause()
        rospy.loginfo("navigate")


    def callback(self, ros_data):
        #rospy.loginfo(self.mode)
        return 0
        if ros_data.status_list is []:
            return 0
        rospy.loginfo("New text from Message:")
        rospy.loginfo(ros_data.status_list[0].text)
        #self.navigate()
        if ros_data.status_list[0].status == 1:
            rospy.loginfo("accepted goal")
            rospy.loginfo(self.mode)

        if ros_data.status_list[0].status == 3 and self.last_status == 1:
            rospy.loginfo("Condition reached")
            self.mode += 1
            self.pause()
            self.navigate()

        rospy.loginfo("last text")
        rospy.loginfo(self.last_text)
        self.last_status = ros_data.status_list[0].status
        self.last_text = ros_data.status_list[0].text
        rospy.loginfo("---------------------------")


    def pause(self):
        rospy.loginfo(self.mode)
        rospy.sleep(2)
        while self.mode > 2:
            rospy.sleep(10)


def main(args):
    ic = waypoints()
    flag = 0
    rospy.init_node('goToGoal', anonymous=True)
    rospy.sleep(2)
    if not flag:
        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        ic.poses[0].header.stamp = now
        ic.waypoint_pub.publish(ic.poses[0])
        #ic.navigate()
        flag = 1
    #rospy.on_shutdown(ic.zeroSpeed)
    #ic.poses[0].header.stamp = now
    #ic.waypoint_pub.publish(ic.poses[0])
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

'''