#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import math
import csv
from geometry_msgs.msg import Pose

###############################################################################
# Global vars
###############################################################################

current_pose_topic = "/cartesian_impedance_controller/current_pose"
desired_pose_topic = "/cartesian_impedance_controller/desired_pose"
pose_data = [0, 0, 0, 0, 0, 0, 0, 0]
pose_data_d = [0, 0, 0, 0, 0, 0, 0, 0]
errors = []
poses = []
t = 0

###############################################################################
# Callback functions
###############################################################################

def current_pose_callback(msg):
  global pose_data, pose_data_d
  pose_data = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
  
def desired_pose_callback(msg):
  global pose_data, pose_data_d, t
  pose_data_d = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
  errors.append({
    't': t, 
    'x': (pose_data_d[0]-pose_data[0]), 
    'y': (pose_data_d[1]-pose_data[1]), 
    'z': (pose_data_d[2]-pose_data[2]), 
    'rx': (pose_data_d[3]-pose_data[3]), 
    'ry': (pose_data_d[4]-pose_data[4]), 
    'rz': (pose_data_d[5]-pose_data[5]), 
    'rw': (pose_data_d[6]-pose_data[6]) 
  })
  poses.append({
    't': t, 
    'x': (pose_data[0]), 
    'y': (pose_data[1]), 
    'z': (pose_data[2]), 
    'rx': (pose_data[3]), 
    'ry': (pose_data[4]), 
    'rz': (pose_data[5]), 
    'rw': (pose_data[6]),
    'xd': (pose_data_d[0]), 
    'yd': (pose_data_d[1]), 
    'zd': (pose_data_d[2]), 
    'rxd': (pose_data_d[3]), 
    'ryd': (pose_data_d[4]), 
    'rzd': (pose_data_d[5]), 
    'rwd': (pose_data_d[6]) 
  })
  t += 1

###############################################################################
# General functions
###############################################################################

def save_data_csv(data):
  path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools"
  with open(path+'/data/trajectory_tracking_errors.csv', 'w', newline='') as csvfile:
    wrt = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    wrt.writerow(['time', 'x', 'y', 'z', 'rx', 'ry', 'rz', 'rw'])
    for i in range(len(data)):
      wrt.writerow([data[i]['t'], data[i]['x'], data[i]['y'], data[i]['z'], data[i]['rx'], data[i]['ry'], data[i]['rz'], data[i]['rw']])

def save_data_errors_plot(data):
  path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools"
  with open(path+'/data/trajectory_tracking_errors_plot.dat', 'w') as f:
    for i in range(len(data)):
      f.write(f"{data[i]['t']} {data[i]['x']} {data[i]['y']} {data[i]['z']} {data[i]['rx']} {data[i]['ry']} {data[i]['rz']} {data[i]['rw']}\n")

def save_data_plot(data):
  path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools"
  with open(path+'/data/trajectory_tracking_plot.dat', 'w') as f:
    for i in range(len(data)):
      f.write(f"{data[i]['t']} {data[i]['x']} {data[i]['y']} {data[i]['z']} {data[i]['rx']} {data[i]['ry']} {data[i]['rz']} {data[i]['rw']} \
      {data[i]['xd']} {data[i]['yd']} {data[i]['zd']} {data[i]['rxd']} {data[i]['ryd']} {data[i]['rzd']} {data[i]['rwd']}\n")

def calc_rms_error(data):
  x_err = 0
  y_err = 0
  z_err = 0
  rx_err = 0
  ry_err = 0
  rz_err = 0
  rw_err = 0

  for error in data:
    x_err += math.pow(error['x'], 2)
    y_err += math.pow(error['y'], 2)
    z_err += math.pow(error['z'], 2)
    rx_err += math.pow(error['rx'], 2)
    ry_err += math.pow(error['ry'], 2)
    rz_err += math.pow(error['rz'], 2)
    rw_err += math.pow(error['rw'], 2)

  if len(data) > 0:
    print(f"RMSE: x: {round(math.sqrt(x_err/len(data)), 5)}, y: {round(math.sqrt(y_err/len(data)), 5)}, z: {round(math.sqrt(z_err/len(data)), 5)}, rx: {round(math.sqrt(rx_err/len(data)), 5)}, ry: {round(math.sqrt(ry_err/len(data)), 5)}, rz: {round(math.sqrt(rz_err/len(data)), 5)}, rw: {round(math.sqrt(rw_err/len(data)), 5)}")

###############################################################################
# Node
###############################################################################

def node():
  rospy.init_node("trajectory_tracking")

  # Subscribe to the current pose and desired pose topics
  rospy.Subscriber(current_pose_topic, Pose, current_pose_callback)
  rospy.Subscriber(desired_pose_topic, Pose, desired_pose_callback)

  r = rospy.Rate(100) # 100 Hz
  while not rospy.is_shutdown():
    r.sleep()

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    save_data_csv(errors)
    save_data_errors_plot(errors)
    save_data_plot(poses)
    print("Data exported.")
    calc_rms_error(errors)
    sys.exit()