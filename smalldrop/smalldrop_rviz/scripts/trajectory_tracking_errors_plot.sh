#!/bin/bash
kst2 ~/ROS/tese/src/smalldrop/smalldrop_rviz/data/trajectory_tracking_errors_plot.dat \
-T 1 --xlabel "Samples" -x 1 --ylabel "Pos X (m)" -P 1 -y 2 \
--xlabel "Samples" -x 1 --ylabel "Pos Y (m)" -P 2 -y 3 \
--xlabel "Samples" -x 1 --ylabel "Pos Z (m)" -P 3 -y 4 \
-T 2 --xlabel "Samples" -x 1 --ylabel "Rot X (°)" -P 4 -y 5 \
--xlabel "Samples" -x 1 --ylabel "Rot Y (°)" -P 5 -y 6 \
--xlabel "Samples" -x 1 --ylabel "Rot Z (°)" -P 6 -y 7 \
;