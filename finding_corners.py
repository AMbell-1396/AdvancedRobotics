#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Libraries
import argparse
import numpy as np
import os, sys
from numpy import linalg as LA
from numpy import linalg as la
import math
import random

# Instantiate CvBridge
bridge = CvBridge()


def image_callback(msg):
    #print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)
    # show image
    #show_image(cv2_img)
    full_process(cv2_img)
    
        
def show_image(img):
     cv2.imshow("Image Window", img)
     cv2.waitKey(3)
        
# @brief Function for converting gray scale to binary
#  @param Matrix
#  @return Matrix
def binary(mat):
    for row in range(0,len(mat)):
        for col in range(0,len(mat[0])):
            if (mat[row,col]>90):
                mat[row,col]=1
            else:
                mat[row,col]=0
    return mat

# @brief To give 4 points in Edgedetection
#  @param Image and old_ctr points
#  @return corners
def Edgedetection(image,old_ctr):

    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray,3)
    (T, thresh) = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
    #show_image(thresh)
    contours, hierarchy=cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ctr=[]
    for j, cnt in zip(hierarchy[0], contours):
        cnt_len = cv2.arcLength(cnt,True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len,True)
        if cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt) and len(cnt) == 4  :
            cnt=cnt.reshape(-1,2)
            if j[0] == -1 and j[1] == -1 and j[3] != -1:
                ctr.append(cnt)
        #print(np.shape(ctr))
        old_ctr=ctr
    return ctr

    
# @brief To find get grid of tag image
#  @param image , corners
#  @return tag image and gray scale image
def perspective_for_tag(ctr,image):
    dst1 = np.array([
        [0, 0],
        [100, 0],
        [100, 100],
        [0, 100]], dtype = "float32")

    M1,status = cv2.findHomography(ctr[0], dst1)
    warp1 = cv2.warpPerspective(image.copy(), M1, (100,100))
    warp2=cv2.medianBlur(warp1,3)
    #warp2= warp1-warp1_5

    tag_image=cv2.resize(warp2, dsize=None, fx=0.09, fy=0.09)
    return tag_image,warp2
    
# @brief To find tag ID and tag image
#  @param image , corners
#  @return tag and tag_id
def Tag_id_detection(ctr,tag_image):
    gray = cv2.cvtColor(tag_image,cv2.COLOR_BGR2GRAY)
    pixel_value=binary(gray)
    #print(pixel_value,'Tag Value')
    status=0
    A_ctr=ctr[0][0]
    #print(A_ctr,'ctr A')
    B_ctr=ctr[0][1]
    #print(B_ctr,'ctr B')
    C_ctr=ctr[0][2]
    #print(C_ctr,'ctr B')
    D_ctr=ctr[0][3]
    #print(D_ctr,'ctr C')
    if (pixel_value[2,2] == 0):
        L1=A_ctr
        L2=B_ctr
        L3=C_ctr
        L4=D_ctr
        status=0
        one = pixel_value[5,5]
        two = pixel_value[5,3]
        three = pixel_value[3,3]
        four = pixel_value[3,5]

    elif pixel_value[6,2]==0:
        L1=D_ctr
        L2=A_ctr
        L3=B_ctr
        L4=C_ctr
        status=1
        one = pixel_value[3,5]
        two = pixel_value[5,5]
        three = pixel_value[5,3]
        four = pixel_value[3,3]

    elif pixel_value[6,6] == 0:
        L1=C_ctr
        L2=D_ctr
        L3=A_ctr
        L4=B_ctr
        status=2
        one = pixel_value[3,3]
        two = pixel_value[3,5]
        three = pixel_value[5,5]
        four = pixel_value[5,3]

    elif pixel_value[2,6] == 0:
        L1=B_ctr
        L2=C_ctr
        L3=D_ctr
        L4=A_ctr
        status=3
        one = pixel_value[5,3]
        two = pixel_value[3,3]
        three = pixel_value[3,5]
        four = pixel_value[5,5]

    else:
        L1=A_ctr
        L2=B_ctr
        L3=C_ctr
        L4=D_ctr
        one = pixel_value[5,5]
        two = pixel_value[5,3]
        three = pixel_value[3,3]
        four = pixel_value[3,5]


    new_ctr=np.array([[L1,L2,L3,L4]])

    #print(new_ctr,'new_ctr')

    tag_id = four*8 + three*4 + two*2 + one*1
    print('Tag id value will be',tag_id)
    return new_ctr,tag_id


#--------------------------------------------------------------
    
def full_process(image):
    corners=Edgedetection(image,0)
    #img = cv2.drawContours(image, corners,0,(0,255,0),1)
    if(len(corners)==0):
        corners=0
	
    tag_image,Tag=perspective_for_tag(corners,image)
    new_corners,tag_id=Tag_id_detection(corners,tag_image)
    print(new_corners)
    
    #pubImage = rospy.Publisher('image_converted', Image)
    #pubImage.publish(image)
    pubCorners = rospy.Publisher('corners', Float64MultiArray, queue_size =10)
    corner_msg = Float64MultiArray()
    corner_msg.data = new_corners
    pubCorners.publish(corner_msg)
    



def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    #cv2.namedWindow("Image Window", 1)
    
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
