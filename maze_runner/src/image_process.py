#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
import cv2
import csv
import numpy as np
import imutils
import os
import my_kmeans as kmeans
from cv_bridge import CvBridge
from collections import Counter

bridge = CvBridge()

#global count
#count = 400

def get_image(CompressedImage):
    imgBGR = bridge.compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
    test_img = np.array(imgBGR)

    test_img_resized = cv2.resize(test_img, (33, 25))
    test_img_resized = test_img_resized.flatten().reshape(1, 33 * 25 * 3)
    test_img_resized = test_img_resized.astype(np.float32)

    cropped_img_list,box_list = kmeans.crop_image(test_img)
    nearest_dist = float('inf')
    k = 21
    #print(len(cropped_img_list))
    box_index = 0
    box_count = 0
    if cropped_img_list == []:
        ret, results, final_neighbours, final_dist = knn.findNearest(test_img_resized, k)
        if final_dist[0][0] > 1000000:
            ret = 0
    for cropped_img in cropped_img_list:
        #global count
        #cv2.imwrite("/home/hal/catkin7785/src/team1_maze_runner/src/2021imgs/{}.jpg".format(count),cropped_img)
        #count += 1
        #if count > 600:
            #count = 400
        cropped_test_img = cv2.resize(cropped_img ,(33,25))
        cropped_test_img = cropped_test_img.flatten().reshape(1, 33 * 25 * 3)
        cropped_test_img = cropped_test_img.astype(np.float32)
        one_ret, results, neighbours, dist = knn.findNearest(cropped_test_img, k)  

        if dist[0][0] > 1000000:
            one_ret = 0
        dist_mean = np.mean(dist[0][0:5])
        if dist_mean < nearest_dist:
            ret = one_ret
            nearest_dist = dist_mean
            box_index = box_count
            final_neighbours = neighbours
            final_dist = dist
        box_count += 1

    img_track = test_img.copy()
    light_house = test_img.shape[1] / 2.0
    box_on_edge = 0
    if len(cropped_img_list) != 0:
        box = box_list[box_index]
        light_house = (box[0][0] + box[1][0]) / (2.0 * test_img.shape[1])
        if box[0][0] > 400 or box[0][0] < 10 or box[1][0] > 400 or box[1][0] < 10:
            box_on_edge = 1
        cv2.rectangle(img_track, box[0], box[1], (255,0,0), 1, 8, 0)
        cv2.putText(img_track, "No." + str(ret) + " pos " + str(light_house), box[0], cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
    
    img_track_pub = bridge.cv2_to_compressed_imgmsg(img_track)
    pubDebug.publish(img_track_pub)
    
    image_sign = Point()
    image_sign.z = box_on_edge
    
    image_sign.x = ret
    image_sign.y = light_house
    pub.publish(image_sign)
        

    






def train_knn(imageDirectory):

    with open(imageDirectory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        lines = list(reader)

    # this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
    train = np.array([np.array(cv2.resize(cv2.imread(imageDirectory +lines[i][0]+".jpg"),(33,25))) for i in range(len(lines))])

    # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
    train_data = train.flatten().reshape(len(lines), 33*25 * 3)
    train_data = train_data.astype(np.float32)

    # read in training labels
    train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])


    ### Train classifier
    global knn
    knn = cv2.ml.KNearest_create()
    knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)




def Init():
    imageDirectory = '/home/hal/Gatech/IntroRoboResearch/Lab6/2021imgs/mod_train_images/'
    train_knn(imageDirectory)

    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, get_image, queue_size=1, buff_size=2**24)
    
    #Initializate the node and gives a name, in this case, 'find_ball'
    rospy.init_node('image_process', anonymous=True)

    #Create a publisher that will be publishing Geometric message Points
    global pub
    pub = rospy.Publisher('/image_sign', Point, queue_size=10)

    #Create a debug publisher for image
    global pubDebug
    pubDebug = rospy.Publisher('trackImage/compressed', CompressedImage, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass
