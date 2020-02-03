#!/bin/bash
kst2 ~/ROS/tese/src/panda_3dbioprint_debug_tools/data/cartesian_profile.dat \
--xlabel "Time (s)" -x 1 --ylabel "Position (rad)" -P 1 -y 2 -y 5 -y 8 \
--xlabel "Time (s)" -x 1 --ylabel "Velocity (rad/s)" -P 2 -y 3 -y 6 -y 9 \
--xlabel "Time (s)" -x 1 --ylabel "Acceleration (rad/s^2)" -P 3 -y 4 -y 7 -y 10 \
--xlabel "Time (s)" -x 1 --ylabel "Orientation (quat)" -P 4 -y 11 -y 14 -y 17 -y 20;
# Velocities quat -y 12 -y 15 -y 18 -y 21 \
# Accelerations quat -y 13 -y 16 -y 19 -y 22 \
# kst2 ~/ROS/tese/src/panda_3dbioprint_debug_tools/data/cartesian_trajectory_profiles.kst