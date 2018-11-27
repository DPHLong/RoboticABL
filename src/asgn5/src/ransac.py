#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt



class asgn5:

  def __init__(self):
    #Publish grey image here
    self.image_pub_grey = rospy.Publisher("/image_processing/grey_img",Image, queue_size=1)
    #Publish binary image here
    self.image_pub_bin = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
    #Publish original image with line here
    self.image_pub_lin = rospy.Publisher("/image_processing/lin_img",Image, queue_size=1)

    self.bridge = CvBridge()
    #Get rgb image from here
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def is_inlier(self,m, c, point):
    max_dist = 15.0
    return self.line_point_distance(m, c, point) < max_dist


  def line_point_distance(self,m, c, point):
    px = point[0]
    py = point[1]

    #Formula from https://www.ias.ac.in/article/fulltext/reso/022/07/0705-0714
    dist = abs( (m*px - py + c) / (math.sqrt(1 + m*m)) )
    return dist


  def get_param_estimate(self,points):
    assert(len(points)==2)

    x1 = points[0][0]
    y1 = points[0][1]
    x2 = points[1][0]
    y2 = points[1][1]

    m = float(y2 - y1) / float(x2 - x1)
    c = y1 - m * x1

    return m, c


  def ransac(self,points):
    iterations = 100
    sample_size = 2

    best_inlier_count = 0
    best_m = 0
    best_c = 0

    random.seed(1457)

    for i in range(iterations):
      hypothetical_inliers = random.sample(points, sample_size)
      m, c = self.get_param_estimate(hypothetical_inliers)
      inlier_count = 0

      for j in range(len(points)):
        if self.is_inlier(m, c, points[j]): inlier_count += 1

        if inlier_count > best_inlier_count:
          best_inlier_count = inlier_count
          best_m = m
          best_c = c

    return best_m, best_c


  def callback(self,data):
    print("Recevied Image")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #Image to greyscale
    image_grey=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
   
    #self.image_pub_grey.publish(self.bridge.cv2_to_imgmsg(image_grey, "mono8"))

    #Image to binary
    bi_gray_max = 255
    bi_gray_min = 208
    ret, image_bin=cv2.threshold(image_grey, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);
    #self.image_pub_bin.publish(self.bridge.cv2_to_imgmsg(image_bin, "mono8"))

    cut_x = 320
    left_image=image_bin[0:480, 0:cut_x]
    right_image=image_bin[0:480, cut_x:640]

    #Line 1
    m1, c1 = self.get_avg_line(left_image)
    print("Linear Function 1: f1(x)=" + str(m1) + "x + " + str(c1))
    max_x = image_bin.shape[0]
    p1_l1 = (int(c1), 0)
    p2_l1 = (int(m1*(max_x-1)+c1), int(max_x-1))

    #Line 2
    m2, c2 = self.get_avg_line(right_image)
    print("Linear Function 2: f2(x)=" + str(m2) + "x + " + str(c2))
    p1_l2 = (int(c2) + cut_x, 0)
    p2_l2 = (int(m2*(max_x-1)+c2) + cut_x, int(max_x-1))

    image_line = cv2.line(cv_image, p2_l1, p1_l1, (0, 0, 255), 3)
    image_line = cv2.line(image_line, p2_l2, p1_l2, (0, 0, 255), 3)

    self.image_pub_lin.publish(self.bridge.cv2_to_imgmsg(image_line, "bgr8"))

  def get_avg_line(self, image):

    #Save white points into array
    white_pixels = []
    for x in range(image.shape[0]):
      for y in range(image.shape[1]):
        if (image[x,y] > 0): 
          white_pixels.append([x, y])

    m, c = self.ransac(white_pixels)
    return m, c
      

def main(args):
  print("Starting Node...")
  rospy.init_node('asgn5', anonymous=True)
  ic = asgn5()
  print("Node Started.")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
