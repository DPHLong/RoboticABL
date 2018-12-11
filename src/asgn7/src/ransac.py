#!/usr/bin/env python

import numpy as np
import math
import random

max_dist = 1.0
iterations = 100
sample_size = 2

def get_distance(x1, y1, x2, y2):
  return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

def set_params(max_dist_, iterations_, sample_size_=2):
  global max_dist, iterations, sample_size
  print("Ransac parameters: max_dist: " + str(max_dist_) + ",  iterations: " + str(iterations_))
  max_dist = max_dist_
  iterations = iterations_
  sample_size = sample_size_

def remove_inliers(m, c, points):
  ret = []
  for p in points:
    if not is_inlier(m, c, p): ret.append(p)
  return ret

def is_inlier(m, c, point):
  global max_dist
  return line_point_distance(m, c, point) < max_dist


def line_point_distance(m, c, point):
  px = point[0]
  py = point[1]

  #Formula from https://www.ias.ac.in/article/fulltext/reso/022/07/0705-0714
  dist = abs( (m*px - py + c) / (math.sqrt(1 + m*m)) )
  return dist


def get_param_estimate(points):
  assert(len(points)==2)

  x1 = points[0][0]
  y1 = points[0][1]
  x2 = points[1][0]
  y2 = points[1][1]

  if (x2 - x1)==0: m = float(y2 - y1) / 0.0001
  else: m = float(y2 - y1) / float(x2 - x1)
  c = y1 - m * x1

  return m, c


def ransac(points):
  global iterations, sample_size

  best_inlier_count = 0
  best_m = 0
  best_c = 0

  random.seed(1457)

  for i in range(iterations):
    hypothetical_inliers = random.sample(points, sample_size)
    m, c = get_param_estimate(hypothetical_inliers)
    inlier_count = 0

    for j in range(len(points)):
      if is_inlier(m, c, points[j]): inlier_count += 1

      if inlier_count > best_inlier_count:
        best_inlier_count = inlier_count
        best_m = m
        best_c = c

  return best_m, best_c

