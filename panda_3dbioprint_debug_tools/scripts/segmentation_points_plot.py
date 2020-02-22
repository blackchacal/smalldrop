#!/usr/bin/env python3

import sys
import rospy
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
import numpy as np
import csv

###############################################################################
# General Functions
###############################################################################

def import_data(filepath):
  with open(filepath, newline='\n') as csvfile:
    xs = []
    ys = []
    zs = []
    oxs = []
    oys = []
    ozs = []
    first = True

    reader = csv.reader(csvfile, delimiter=' ', quotechar='"')
    for row in reader:
      if not first:
        xs.append(round(float(row[0]), 3))
        ys.append(round(float(row[1]), 3))
        zs.append(round(float(row[2]), 3))
        oxs.append(round(float(row[3]), 3))
        oys.append(round(float(row[4]), 3))
        ozs.append(round(float(row[5]), 3))
      first = False

  return [np.array(xs), np.array(ys), np.array(zs), np.array(oxs), np.array(oys), np.array(ozs)]

###############################################################################
# Node
###############################################################################

def node():
  rospy.init_node("segmentation_points_plot")

  # Call rosrun package node.py _is_3d:=<val> to start acquiring data immediately
  is_3D = bool(rospy.get_param('~is_3d', 0)) # Default False
  if rospy.has_param('~is_3d'):
    rospy.delete_param('~is_3d')

  fig = plt.figure()
  if is_3D:
    ax = fig.add_subplot(111, projection='3d')
  else:
    ax = fig.add_subplot(111)


  # Read points data
  path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data"
  xs, ys, zs, oxs, oys, ozs = import_data(path+"/segmentation_points.dat")

  if is_3D:
    # ax.quiver(xs, ys, zs, oxs, oys, ozs, length=0.001, normalize=True)
    ax.scatter(xs, ys, zs, marker='o')
  else:
    ax.scatter(xs, ys, marker='o')

  # ax.set_xlim(0, 0.5)
  # ax.set_ylim(0, 0.5)
  # ax.set_zlim(0, 0.5)
  if not is_3D:
    ax.set_aspect('equal', adjustable='box')
  ax.set_xlabel('X (m)')
  ax.set_ylabel('Y (m)')
  if is_3D:
    ax.set_zlabel('Z (m)')

  plt.show()

  freq = 10 # 10 Hz
  r = rospy.Rate(freq)
  while not rospy.is_shutdown():
    r.sleep()

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    print("Program terminated.")
    sys.exit()