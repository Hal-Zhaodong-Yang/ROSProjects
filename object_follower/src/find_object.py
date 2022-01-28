#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
import cv2 as cv
import numpy as np
import imutils
import os

def findDiamond(img_rgb):
    path = '/home/hal/catkin7785/src/team1_object_follower/src/diamond.jpg'
    #rospy.loginfo("************************************************")
    isExist = os.path.exists(path)
    #rospy.loginfo(isExist)
    template = cv.imread(path)
    #rospy.loginfo(template)
    #rospy.loginfo(img_rgb)
    template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    #w, h = template.shape[::-1]
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
    w, h = img_gray.shape[::-1]
    wi, hi = template.shape[::-1]
    for scale in np.linspace(1.0, 2.0, 3):  #change the size of the picture while matching the template
        resized = imutils.resize(template, width = int(template.shape[1]*scale))
        #r = img_gray.shape[1] / float(resized.shape[1])
        if resized.shape[0] > h or resized.shape[1] > w:
            break
        res = cv.matchTemplate(img_gray, resized, cv.TM_CCOEFF_NORMED)
        threshold = 0.88
        min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
        rospy.loginfo("max_val:")
        rospy.loginfo(max_val)
        if max_val >= threshold:
            #rospy.loginfo("find_object_runs_smoothly_and_object_found")
            return [int(max_loc[0]),int(max_loc[1]), w]
    #rospy.loginfo("seems_no_object")
    return [w / 2, h / 2, w]

class find_object:
    def __init__(self):
        self.img_sub = rospy.Subscriber("/raspicam_node/image/compressed",
            CompressedImage,self.callback, queue_size = 1, buff_size = 2**24)
        self.point_pub = rospy.Publisher("/geometry_msgs/Point", Point,queue_size = 1)

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        position = findDiamond(image_np)
        rospy.loginfo("return_position:")
        rospy.loginfo(position)
	rospy.loginfo("image name:")
	rospy.loginfo(ros_data.header)
        pos3 = Point()
        pos3.x = position[0]
        pos3.y = position[1]
        pos3.z = position[2]
        self.point_pub.publish(pos3)

def main(args):
    ic = find_object()
    rospy.init_node('find_object', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
