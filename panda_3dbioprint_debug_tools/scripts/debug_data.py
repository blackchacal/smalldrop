#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import math
import csv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench
from panda_3dbioprint_simulation.msg import Tau
from panda_3dbioprint_simulation.msg import TrackingError

###############################################################################
# Global vars
###############################################################################

current_pose_topic = "/cartesian_impedance_controller/current_pose"
desired_pose_topic = "/cartesian_impedance_controller/desired_pose"
tau_topic = "/cartesian_impedance_controller/tau"
wrench_topic = "/cartesian_impedance_controller/wrench"
error_topic = "/cartesian_impedance_controller/error"
data = {
  'time': [],
  'pose': [],
  'pose_d': [],
  'error': [],
  'wrench': [],
  'tau_d': [],
  'tau_task': [],
  'tau_null': []  
}
t = 0
save = False
atonce = False
fh = None
pose_check = False
pose_d_check = False
error_check = False
tau_check = False
wrench_check = False

###############################################################################
# Callback functions
###############################################################################

def current_pose_callback(msg):
  global data, save, t
  pose_data = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
  if save:
    data['pose'].append(pose_data)
    data['time'].append(t)
    t += 1
    pose_check = True
  
  
def desired_pose_callback(msg):
  global data, save
  if not save:
    save = True
  pose_d = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
  data['pose_d'].append(pose_d)
  pose_d_check = True

def tau_callback(msg):
  global data, save
  tau_d = [msg.tau_d[0], msg.tau_d[1], msg.tau_d[2], msg.tau_d[3], msg.tau_d[4], msg.tau_d[5], msg.tau_d[6]]
  tau_task = [msg.tau_task[0], msg.tau_task[1], msg.tau_task[2], msg.tau_task[3], msg.tau_task[4], msg.tau_task[5], msg.tau_task[6]]
  tau_null = [msg.tau_null[0], msg.tau_null[1], msg.tau_null[2], msg.tau_null[3], msg.tau_null[4], msg.tau_null[5], msg.tau_null[6]]
  if save:
    data['tau_d'].append(tau_d)
    data['tau_task'].append(tau_task)
    data['tau_null'].append(tau_null)
    tau_check = True

def wrench_callback(msg):
  global data, save
  wrench = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]
  if save:
    data['wrench'].append(wrench)
    wrench_check = True

def tracking_errors_callback(msg):
  global data, save
  error = [msg.error[0], msg.error[1], msg.error[2], msg.error[3], msg.error[4], msg.error[5]]
  if save:
    data['error'].append(error)
    error_check = True

###############################################################################
# General functions
###############################################################################

def open_file():
  path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools"
  f = open(path+'/data/debug_data.dat', 'w')
  f.write("time px py pz ox oy oz px_d py_d pz_d ox_d oy_d oz_d px_err py_err pz_err ox_err oy_err oz_err \
  wfx wfy wfz wtx wty wtz tau_d_1 tau_d_2 tau_d_3 tau_d_4 tau_d_5 tau_d_6 tau_d_7 \
  tau_task_1 tau_task_2 tau_task_3 tau_task_4 tau_task_5 tau_task_6 tau_task_7 \
  tau_null_1 tau_null_2 tau_null_3 tau_null_4 tau_null_5 tau_null_6 tau_null_7 \n")

  return f

def write_data(fh, data):
  print(f"{len(data['time'])} {len(data['pose'])} {len(data['pose_d'])} {len(data['error'])} {len(data['wrench'])} {len(data['tau_d'])} {len(data['tau_task'])} {len(data['tau_null'])}")
  if pose_check and pose_d_check and tau_check and error_check and wrench_check:
    for i in range(len(data['time'])):
      fh.write(f"{data['time'][i]} {data['pose'][i][0]} {data['pose'][i][1]} {data['pose'][i][2]} {data['pose'][i][3]} {data['pose'][i][4]} {data['pose'][i][5]} {data['pose'][i][6]} \
      {data['pose_d'][i][0]} {data['pose_d'][i][1]} {data['pose_d'][i][2]} {data['pose_d'][i][3]} {data['pose_d'][i][4]} {data['pose_d'][i][5]} {data['pose_d'][i][6]} \
      {data['error'][i][0]} {data['error'][i][1]} {data['error'][i][2]} {data['error'][i][3]} {data['error'][i][4]} {data['error'][i][5]} \
      {data['wrench'][i][0]} {data['wrench'][i][1]} {data['wrench'][i][2]} {data['wrench'][i][3]} {data['wrench'][i][4]} {data['wrench'][i][5]} \
      {data['tau_d'][i][0]} {data['tau_d'][i][1]} {data['tau_d'][i][2]} {data['tau_d'][i][3]} {data['tau_d'][i][4]} {data['tau_d'][i][5]} {data['tau_d'][i][6]} \
      {data['tau_task'][i][0]} {data['tau_task'][i][1]} {data['tau_task'][i][2]} {data['tau_task'][i][3]} {data['tau_task'][i][4]} {data['tau_task'][i][5]} {data['tau_task'][i][6]} \
      {data['tau_null'][i][0]} {data['tau_null'][i][1]} {data['tau_null'][i][2]} {data['tau_null'][i][3]} {data['tau_null'][i][4]} {data['tau_null'][i][5]} {data['tau_null'][i][6]}\n")

def save_data(data):
    f = open_file()
    write_data(f, data)
    f.close()

def calc_rms_error(data):
  x_err = 0
  y_err = 0
  z_err = 0
  ox_err = 0
  oy_err = 0
  oz_err = 0

  for error in data['error']:
    x_err += math.pow(error[0], 2)
    y_err += math.pow(error[1], 2)
    z_err += math.pow(error[2], 2)
    ox_err += math.pow(error[3], 2)
    oy_err += math.pow(error[4], 2)
    oz_err += math.pow(error[5], 2)

  if len(data['time']) > 0:
    print(f"RMSE: x: {round(math.sqrt(x_err/len(data)), 5)}, y: {round(math.sqrt(y_err/len(data)), 5)}, z: {round(math.sqrt(z_err/len(data)), 5)}, rx: {round(math.sqrt(ox_err/len(data)), 5)}, ry: {round(math.sqrt(oy_err/len(data)), 5)}, rz: {round(math.sqrt(oz_err/len(data)), 5)}")

###############################################################################
# Node
###############################################################################

def node():
  global fh, save

  rospy.init_node("debug_data")

  # Call rosrun package node.py _atonce:=<val> to start acquiring data immediately
  atonce = bool(rospy.get_param('~atonce', 0)) # Default False
  if rospy.has_param('~atonce'):
    rospy.delete_param('~atonce')

  if atonce:
    print("Starting logging data...")
    save = True
    fh = open_file()

  # Subscribe to the current pose and desired pose topics
  rospy.Subscriber(current_pose_topic, Pose, current_pose_callback)
  rospy.Subscriber(desired_pose_topic, Pose, desired_pose_callback)
  rospy.Subscriber(tau_topic, Tau, tau_callback)
  rospy.Subscriber(wrench_topic, Wrench, wrench_callback)
  rospy.Subscriber(error_topic, TrackingError, tracking_errors_callback)

  r = rospy.Rate(100) # 100 Hz
  while not rospy.is_shutdown():
    if atonce:
      write_data(fh, data)
    r.sleep()

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    if type(fh) is file:
      fh.close()
    if not atonce:
      save_data(data)
    print("Data exported.")
    calc_rms_error(data)
    sys.exit()