# Message type for robot arm controller torques.
# joint_name: List of joint names
# tau_d: List of desired torques for each joint as calculated by the controller.
# tau_task: List of task specific torques for each joint as calculated by the controller.
# tau_null: List of null space torques for each joint as calculated by the controller.

string[] joint_name
float64[] tau_d
float64[] tau_task
float64[] tau_null 