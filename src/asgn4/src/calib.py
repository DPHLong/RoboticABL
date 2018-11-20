#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt



class image_converter:

  def __init__(self):
    #Publish grey image here
    self.image_pub_grey = rospy.Publisher("/image_processing/grey_img",Image, queue_size=1)
    #Publish binary image here
    self.image_pub_bin = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    #Get rgb image from here
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    print("Recevied Image")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #Image to greyscale
    image_grey=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #Crop image to relevant area
    min_y = 220
    max_y = 460
    min_x = 120
    max_x = 560
    image_grey=image_grey[min_y:max_y, min_x:max_x]
   
    try:
      self.image_pub_grey.publish(self.bridge.cv2_to_imgmsg(image_grey, "mono8"))
    except CvBridgeError as e:
      print(e)


    #Image to binary
    bi_gray_max = 255
    bi_gray_min = 200
    ret, image_bin=cv2.threshold(image_grey, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
    try:
      self.image_pub_bin.publish(self.bridge.cv2_to_imgmsg(image_bin, "mono8"))
    except CvBridgeError as e:
      print(e)


    #Save white points into array
    min_dist = 30.0
    white_points = []
    for x in range(image_bin.shape[0]):
      for y in range(image_bin.shape[1]):
        if (image_bin[x,y] > 0):
          included = False
          for [x2, y2] in white_points:
            if math.sqrt(abs(x - x2)**2 + abs(y - y2)**2) < min_dist:
              included = True
          if not included: 
            white_points.append([x, y])
            print("White Point on binary image at: " + str(x) + " : " + str(y))

    #Extrinsic Parameters
    fx = 614.1699
    fy = 614.9002
    cx = 329.9491
    cy = 237.2788
    camera_mat = np.zeros((3,3,1))
    camera_mat[:,:,0] = np.array([ [fx,0,cx], [0,fy,cy], [0,0,1] ])

    k1 = 0.1115
    k2 = -0.1089
    p1 = 0
    p2 = 0
    dist_coeffs = np.zeros((4,1))
    dist_coeffs[:,0] = np.array([[k1,k2,p1,p2]])

    img_points = np.zeros((len(white_points),2,1))
    img_points[:,:,0] = np.array(white_points)

    obj_points = np.zeros((6,3,1)) #Points measured ont he plane (in cm)
    obj_points[:,:,0] = np.array([[00.0, 00.0, 0.0], [29.0, 00.0, 0.0] , [00.0, 40.0, 0.0], [29.0, 40.0, 0.0] , [00.0, 80.0, 0.0], [29.0, 80.0, 0.0]])

    retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_mat, dist_coeffs)
    print("Rotation Vector:")
    print(rvec) #Rotation vector
    print("Translation Vector:")
    print(tvec) #Translation vector

    rmat = np.zeros((3,3))
    cv2.Rodrigues(rvec, rmat, jacobian=0)
    print("Rotation Matrix:")
    print(rmat)

    #Homogenous Transform
    htm = np.zeros((4,4))
    htm = np.array( [ 	[rmat[0,0], rmat[1,0], rmat[2,0], tvec[0]],
			[rmat[0,1], rmat[1,1], rmat[2,1], tvec[1]],
			[rmat[0,2], rmat[1,2], rmat[2,2], tvec[2]],
			[0, 0, 0, 1] ] )
    htm_inv = np.linalg.inv(htm)
    print("Inverse HTM:")
    print(htm_inv)


    #Get euler/angles yaw/pitch/roll
    proj_mat = np.hstack((rmat, tvec))
    euler_angles = cv2.decomposeProjectionMatrix(proj_mat)[6]
    print("Euler Angles:")
    print(euler_angles)
      

def main(args):
  print("Starting Node...")
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  print("Node Started.")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
