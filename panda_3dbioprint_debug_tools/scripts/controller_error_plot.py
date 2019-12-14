#!/usr/bin/env python

import rospy
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Pose

current_pose_topic = "/cartesian_impedance_controller/current_pose"
desired_pose_topic = "/cartesian_impedance_controller/desired_pose"
pos_data = [0, 0, 0]
orient_data = [0, 0, 0, 0]
pos_data_d = [0, 0, 0]
orient_data_d = [0, 0, 0, 0]

###############################################################################
# Classes
###############################################################################

class Plotter():
  def __init__(self):
    self.time = 0
    self.freq = 1

  def setup(self, freq, visibility):
    self.figure, self.ax = plt.subplots(nrows=2, ncols=1, sharex=True)
    self.lines = []
    for i in range(len(self.ax)):
      if i == 1:
        self.lines.append(self.ax[i].plot([],[], 'r-', [],[], 'g-', [],[], 'b-', [],[], 'k-', \
          [],[], 'r--', [],[], 'g--', [],[], 'b--', [],[], 'k--'))
      else:
        self.lines.append(self.ax[i].plot([],[], 'r-', [],[], 'g-', [],[], 'b-', \
          [],[], 'r--', [],[], 'g--', [],[], 'b--'))
      # Autoscale on unknown axis and known lims on the other
      self.ax[i].set_autoscaley_on(True)
      self.ax[i].grid()
      self.ax[i].set_xlabel("Time (s)")

      # Set visible plots
      for j in range(len(visibility[i])):
        self.lines[i][j].set_visible(visibility[i][j])

    self.ax[0].set_ylabel("Position (m)")
    self.ax[1].set_ylabel("Orientation (ND)")
    self.ax[0].set_title("End-Effector Position vs Time")
    self.ax[1].set_title("End-Effector Orientation Quaternion vs Time")
    self.ax[0].legend(["x", "y", "z", "x_d", "y_d", "z_d"])
    self.ax[1].legend(["x", "y", "z", "w", "x_d", "y_d", "z_d", "w_d"])
    self.freq = freq

  def update(self, data):
    """
    data: is an array. 0-position data; 1-orientation data
    """
    for i in range(len(self.ax)):
      for j in range(len(self.lines[i])):
        self.lines[i][j].set_xdata(np.append(self.lines[i][j].get_xdata(), self.time))
        self.lines[i][j].set_ydata(np.append(self.lines[i][j].get_ydata(), data[i][j]))
      # Need both of these in order to rescale
      self.ax[i].relim()
      self.ax[i].autoscale_view()

    # We need to draw *and* flush
    self.figure.tight_layout()
    self.figure.canvas.draw()
    self.figure.canvas.flush_events()
    self.figure.show()
    self.time += 1.0/self.freq

###############################################################################
# Callback functions
###############################################################################

def current_pose_callback(msg):
  global pos_data, orient_data
  pos_data = [msg.position.x, msg.position.y, msg.position.z]
  orient_data = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
  
def desired_pose_callback(msg):
  global pos_data_d, orient_data_d
  pos_data_d = [msg.position.x, msg.position.y, msg.position.z]
  orient_data_d = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

###############################################################################
# General functions
###############################################################################

def set_visibility(str):
  """
  Sets the visibility of the plots.
  To set, use a string with plot codes separated by commas. If the string is empty
  all plots show. If not, only valid plots are shown. If no valid plots are presented
  no plots will be shown.

  Plot codes:
  px - Position x
  py - Position y
  pz - Position z
  pxd - Position x desired
  pyd - Position y desired
  pzd - Position z desired
  ox - Orientation x
  oy - Orientation y
  oz - Orientation z
  ow - Orientation w
  oxd - Orientation x desired
  oyd - Orientation y desired
  ozd - Orientation z desired
  owd - Orientation w desired
  """
  visibility = [ 
    [True,True,True,True,True,True], # position [x,y,z,x_d,y_d,z_d]
    [True,True,True,True,True,True,True,True] # orientation [x,y,z,w,x_d,y_d,z_d,w_d]
  ]

  if str != '':
    # Turn all off
    visibility = [ 
      [False,False,False,False,False,False], # position [x,y,z,x_d,y_d,z_d]
      [False,False,False,False,False,False,False,False] # orientation [x,y,z,w,x_d,y_d,z_d,w_d]
    ]
    visibles = str.split(",")
    for i in range(len(visibles)):
      # Check the ones selected to be on
      if visibles[i] == 'px':
        visibility[0][0] = True
      elif visibles[i] == 'py':
        visibility[0][1] = True
      elif visibles[i] == 'pz':
        visibility[0][2] = True
      elif visibles[i] == 'pxd':
        visibility[0][3] = True
      elif visibles[i] == 'pyd':
        visibility[0][4] = True
      elif visibles[i] == 'pzd':
        visibility[0][5] = True
      elif visibles[i] == 'ox':
        visibility[1][0] = True
      elif visibles[i] == 'oy':
        visibility[1][1] = True
      elif visibles[i] == 'oz':
        visibility[1][2] = True
      elif visibles[i] == 'ow':
        visibility[1][3] = True
      elif visibles[i] == 'oxd':
        visibility[1][4] = True
      elif visibles[i] == 'oyd':
        visibility[1][5] = True
      elif visibles[i] == 'ozd':
        visibility[1][6] = True
      elif visibles[i] == 'owd':
        visibility[1][7] = True

  return visibility

###############################################################################
# Node
###############################################################################

def node():
  global pos_data, orient_data

  rospy.init_node("controller_error_plot")

  # Call rosrun package node.py _freq:=<val> to set plot update frequency
  freq = rospy.get_param('~freq',10) # Default 10 Hz
  if rospy.has_param('~freq'):
    rospy.delete_param('~freq')

  # Call rosrun package node.py _show:='<code>,<code>,...' to set plots visibility
  # Refer to set_visibility() comments.
  visibility = set_visibility(rospy.get_param('~show',""))
  # Remove param so that empty strings, or not using the cmdline arg, always works
  if rospy.has_param('~show'):
    rospy.delete_param('~show')

  p = Plotter()
  p.setup(freq, visibility)

  # Subscribe to the current pose and desired pose topics
  rospy.Subscriber(current_pose_topic, Pose, current_pose_callback)
  rospy.Subscriber(desired_pose_topic, Pose, desired_pose_callback)

  r = rospy.Rate(freq)
  while not rospy.is_shutdown():
    p.update([pos_data+pos_data_d, orient_data+orient_data_d])
    r.sleep()

if __name__ == '__main__':
  try:
    node()
  except rospy.ROSInterruptException:
    print "Program interrupted before completion."