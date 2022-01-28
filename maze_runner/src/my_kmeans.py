#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import sys
import csv

normalize_rx = 255 / 410
normalize_ry = 255 / 308

def get_initial_means(array, k):
    """
    Picks k random points from the 2D array
    (without replacement) to use as initial
    cluster means

    params:
    array = numpy.ndarray[numpy.ndarray[float]] - m x n | datapoints x features

    k = int

    returns:
    initial_means = numpy.ndarray[numpy.ndarray[float]]
    """
    # TODO: finish this function
    k_indices = np.random.choice(array.shape[0],k,replace = False)
    initial_means = array[list(k_indices),]

    return initial_means

def k_means_step(X, k, means):
    """
    A single update/step of the K-means algorithm
    Based on a input X and current mean estimate,
    predict clusters for each of the pixels and
    calculate new means.
    params:
    X = numpy.ndarray[numpy.ndarray[float]] - m x n | pixels x features (already flattened)
    k = int
    means = numpy.ndarray[numpy.ndarray[float]] - k x n

    returns:
    (new_means, clusters)
    new_means = numpy.ndarray[numpy.ndarray[float]] - k x n
    clusters = numpy.ndarray[int] - m sized vector
    """
    # TODO: finish this function
    X = X.reshape((X.shape[0],1,X.shape[1]))
    means = means.reshape((1,means.shape[0],means.shape[1]))
    diff = X - means
    distance = np.sqrt(np.sum(diff * diff, axis = 2))
    clusters = np.argmin(distance,axis = 1)
    new_means = []
    X = X.squeeze()
    for i in range(k):
        new_mean = np.mean(X[np.where(clusters == i)],axis = 0)
        new_means.append(list(new_mean))

    new_means = np.array(new_means)

    return new_means, clusters


def k_means_seperate(image_values, k=3, initial_means=None):
    """
    Separate the provided RGB values into
    k separate clusters using the k-means algorithm,
    then return an updated version of the image
    with the original values replaced with
    the corresponding cluster values.

    params:
    image_values = numpy.ndarray[numpy.ndarray[numpy.ndarray[float]]] - r x c x ch
    k = int
    initial_means = numpy.ndarray[numpy.ndarray[float]] or None

    returns:
    updated_image_values = numpy.ndarray[numpy.ndarray[numpy.ndarray[float]]] - r x c x ch
    """
    # TODO: finish this function

    image_reshaped = image_values.copy().reshape((-1,image_values.shape[2]))
    if initial_means is None:
        means = get_initial_means(image_reshaped, k)
    else:
        means = initial_means
    
    converge = False
    means,last_clusters = k_means_step(image_reshaped, k, means)
    
    while not converge:
        means,clusters = k_means_step(image_reshaped, k, means)
        if np.isnan(np.sum(means)):
            converge = False
            means = get_initial_means(image_reshaped, k)
            means,clusters = k_means_step(image_reshaped, k, means)
        if np.allclose(clusters,last_clusters):
            converge = True
        last_clusters = clusters

    box = []
    updated_image_values = []
    for i in range(k):
        """cluster = image_reshaped[np.where(last_clusters == i)]
        lefttop_x = int(np.min(cluster[:,3]) / normalize_rx)
        lefttop_y = int(np.min(cluster[:,4]) / normalize_ry)
        rightbottom_x = int(np.max(cluster[:,3]) / normalize_rx)
        rightbottom_y = int(np.max(cluster[:,4]) / normalize_ry)
        box.append([(lefttop_x,lefttop_y),(rightbottom_x,rightbottom_y)])"""   
        
        binary = np.zeros((image_values.shape[0] *  image_values.shape[1]),dtype=np.uint8)
        binary[np.where(last_clusters == i)] = 255
        updated_image_values.append(binary.reshape(image_values.shape[0],  image_values.shape[1]))
        #image_reshaped[np.where(last_clusters == i)] = means[i]
        #image_reshaped[np.where(last_clusters == i)] = i * 255 / k

    #print(box)
    #updated_image_values = image_reshaped.reshape((image_values.shape[0],image_values.shape[1],image_values.shape[2]))

    return updated_image_values


def extract_box(binary_list, expand_pixels):

    k = len(binary_list)
    img_w = binary_list[0].shape[1]
    img_h = binary_list[0].shape[0]
    box_list_total = []
    for i in range(k):
        num_labels, labels, stats, centers = cv.connectedComponentsWithStats(binary_list[i])
        box_list_img = []
        for t in range(1, num_labels, 1):
            x, y, w, h, area = stats[t]
            if w < 40 or h < 40:
                continue
            if w > 350 or h > 260:
                continue
            topleft_x = x - np.min([expand_pixels, x])
            topleft_y = y - np.min([expand_pixels, y])
            bottomright_x = x + w + np.min([expand_pixels, img_w - w - x])
            bottomright_y = y + h + np.min([expand_pixels, img_h - y - h])
            
            box = ((topleft_x,topleft_y),(bottomright_x, bottomright_y))
            box_list_img.append(box)
        box_list_total.append(box_list_img)


    return box_list_total


def crop_image(img, k = 3, expand_pixels = 3):

    color_seperated = k_means_seperate(img,k)
    box_list = extract_box(color_seperated,expand_pixels)

    cropped_img_list = []
    new_box_list = []
    for box_list_image in box_list:
        #print(box_list_image)
        if box_list_image == []:
            continue
        #image = np.copy(img)

        for box in box_list_image:
            #print(box)
            #cv.rectangle(image, (box[0][0], box[0][1]), (box[1][0], box[1][1]), (255,0,0), 1, 8, 0)
            cropped_img = img[ box[0][1] : box[1][1], box[0][0] : box[1][0]]
            cropped_img_list.append(cropped_img)
            new_box_list.append(box)

    return cropped_img_list, new_box_list




def main(args):

    #change it every time switching in train and test
    imageDirectory = './2021imgs/test_images/'

    image_name = '190'
    
    k = 3

    img = cv.imread(imageDirectory + image_name + '.jpg')
    img = np.array(img)

    color_seperated = k_means_seperate(img,k)
    print(color_seperated[0].shape)
    cropped_img_list = crop_image(img)
    i = 0
    for cropped_img in cropped_img_list:
        cv.imshow('experiment{}'.format(i), cropped_img)
        cv.imwrite('experiment{}.jpg'.format(i), cropped_img)
        i += 1



    box_list = extract_box(color_seperated,3)
    #print(box_list)
    i = 0
    for box_list_image in box_list:
        #print(box_list_image)
        if box_list_image == []:
            continue
        image = np.copy(img)

        for box in box_list_image:
            #print(box)
            cv.rectangle(image, (box[0][0], box[0][1]), (box[1][0], box[1][1]), (255,0,0), 1, 8, 0)

        print("showed an image once")
        cv.imshow("colored labels{}".format(i), image)
        cv.imshow('binary{}'.format(i),color_seperated[i])
        i += 1

    cv.waitKey(0)
    

if __name__ == '__main__':
    main(sys.argv)