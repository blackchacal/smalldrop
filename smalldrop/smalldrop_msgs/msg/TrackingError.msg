# Message type for robot arm motion tracking error.
# error: List of position and orientation errors. Since the motion space has 6 degrees of 
#        freedom, the list has size 6. It is [err_px, err_py, err_pz, err_ox, err_oy, err_oz].

float64[] error