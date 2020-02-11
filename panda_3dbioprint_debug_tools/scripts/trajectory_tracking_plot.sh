#!/bin/bash
kst2 ~/ROS/tese/src/panda_3dbioprint_debug_tools/data/trajectory_tracking_plot.dat \
--xlabel "Samples" -x 1 --ylabel "Pos X (m)" -P 1 -y 2 -y 9 \
--xlabel "Samples" -x 1 --ylabel "Pos Y (m)" -P 2 -y 3 -y 10 \
--xlabel "Samples" -x 1 --ylabel "Pos Z (m)" -P 3 -y 4 -y 11 \
--xlabel "Samples" -x 1 --ylabel "Rot X ()" -P 4 -y 5 -y 12 \
--xlabel "Samples" -x 1 --ylabel "Rot Y ()" -P 5 -y 6 -y 13 \
--xlabel "Samples" -x 1 --ylabel "Rot Z ()" -P 6 -y 7 -y 14 \
--xlabel "Samples" -x 1 --ylabel "Rot W ()" -P 7 -y 8 -y 15 \
;