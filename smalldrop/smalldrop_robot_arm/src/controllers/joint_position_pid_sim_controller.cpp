// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

#include <smalldrop_robot_arm/joint_position_pid_sim_controller.h>

namespace smalldrop
{
namespace smalldrop_robot_arm
{
/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

bool JointPositionPIDSimController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                         ros::NodeHandle &controller_nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!root_nh.getParam("/joint_position_pid_sim_controller/arm_id", arm_id))
  {
    ROS_ERROR_STREAM("JointPositionPIDSimController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!root_nh.getParam("/joint_position_pid_sim_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("JointPositionPIDSimController: Invalid or no joint_names parameters "
              "provided, aborting controller init!");
    return false;
  }
  n_joints_ = joint_names.size();

  // Verify effort joint interface
  std::cout << "Verifying effort joint interface" << std::endl;
  auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM("JointPositionPIDSimController: Error getting effort joint "
                     "interface from hardware");
    return false;
  }

  // Verify joint handles
  std::cout << "Verifying joint handles" << std::endl;
  for (size_t i = 0; i < n_joints_; ++i)
  {
    try
    {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("JointPositionPIDSimController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Parse KDL tree
  std::string robot_desc_string;
  root_nh.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, k_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree.");
    return false;
  }

  // Get chain from kdl tree
  if (!k_tree_.getChain("panda_link0", "panda_EE", k_chain_))
  {
    ROS_ERROR("Failed to get chain from kdl tree!");
  }

  n_joints_ = k_chain_.getNrOfJoints();

  // Load joint model from urdf to get joint limits
  urdf::Model model;
  if (model.initParam("/robot_description"))
  {
    robot_joints_ = model.joints_;
  }
  else
  {
    ROS_ERROR("Unable to read the robot model from URDF.");
  }

  // Setup dynamic reconfigure
  dyn_config_gains_node_ = ros::NodeHandle("joint_position_pid_sim_controller/gains");
  dyn_config_gains_param_ =
      std::make_unique<dynamic_reconfigure::Server<::smalldrop_robot_arm::JointPositionPIDSimControllerConfig>>(
          dyn_config_gains_node_);
  dyn_config_gains_param_->setCallback(
      boost::bind(&JointPositionPIDSimController::updateDynamicConfigGainsCallback, this, _1, _2));

  setupPublishersAndSubscribers(controller_nh);

  // Setup real and user defined joint limits
  max_joint_limits_ << robot_joints_[joint_handles_[0].getName()].get()->limits.get()->upper,
      robot_joints_[joint_handles_[1].getName()].get()->limits.get()->upper,
      robot_joints_[joint_handles_[2].getName()].get()->limits.get()->upper,
      robot_joints_[joint_handles_[3].getName()].get()->limits.get()->upper,
      robot_joints_[joint_handles_[4].getName()].get()->limits.get()->upper,
      robot_joints_[joint_handles_[5].getName()].get()->limits.get()->upper,
      robot_joints_[joint_handles_[6].getName()].get()->limits.get()->upper;
  min_joint_limits_ << robot_joints_[joint_handles_[0].getName()].get()->limits.get()->lower,
      robot_joints_[joint_handles_[1].getName()].get()->limits.get()->lower,
      robot_joints_[joint_handles_[2].getName()].get()->limits.get()->lower,
      robot_joints_[joint_handles_[3].getName()].get()->limits.get()->lower,
      robot_joints_[joint_handles_[4].getName()].get()->limits.get()->lower,
      robot_joints_[joint_handles_[5].getName()].get()->limits.get()->lower,
      robot_joints_[joint_handles_[6].getName()].get()->limits.get()->lower;

  // User defined joint limits
  std::vector<double> min_limits;
  std::vector<double> min_limits_defaults(min_joint_limits_.data(), min_joint_limits_.data()+min_joint_limits_.size());
  std::vector<double> max_limits;
  std::vector<double> max_limits_defaults(max_joint_limits_.data(), max_joint_limits_.data()+max_joint_limits_.size());
  root_nh.param("smalldrop/robot/workspace/min_joint_limits", min_limits, min_limits_defaults);
  root_nh.param("smalldrop/robot/workspace/max_joint_limits", max_limits, max_limits_defaults);

  // Limit the parameter values to real joint limits
  for (size_t i = 0; i < min_limits.size(); i++)
  {
    if (min_limits[i] < min_joint_limits_(i))
      min_user_joint_limits_(i) = min_joint_limits_(i);
    else
      min_user_joint_limits_(i) = min_limits[i];
    
    if (max_limits[i] > max_joint_limits_(i))
      max_user_joint_limits_(i) = max_joint_limits_(i);
    else
      max_user_joint_limits_(i) = max_limits[i];
  }
  
  return true;
}

void JointPositionPIDSimController::starting(const ros::Time &time)
{
  // Set desired joint positions
  q_d_ << 0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4;
  qdot_d_ << 0, 0, 0, 0, 0, 0, 0;
  q_d_target_ = q_d_;

  // Initialize errors
  error_ << 0, 0, 0, 0, 0, 0, 0;
  vel_error_ << 0, 0, 0, 0, 0, 0, 0;
  error_accum_ << 0, 0, 0, 0, 0, 0, 0;
}

void JointPositionPIDSimController::update(const ros::Time &time, const ros::Duration &period)
{
  // Get joint states
  for (size_t i = 0; i < n_joints_; ++i) {
    q_(i) = joint_handles_[i].getPosition();
    qdot_(i) = joint_handles_[i].getVelocity();
  }

  // Calculate X = f(q) using forward kinematics (FK)
  Eigen::Matrix4d T0ee;
  if (!fk(q_, T0ee))
    ROS_ERROR("Cannot get forward kinematics.");

  Eigen::Affine3d transform(T0ee);
  Eigen::Vector3d X0ee(transform.translation());
  Eigen::Matrix3d R0ee(transform.rotation());

  // Publish current pose and joint configurations
  publishCurrentJointPositions(q_);
  publishCurrentPose(X0ee, R0ee);

  // ---------------------------------------------------------------------------
  // Set the controller gains
  // ---------------------------------------------------------------------------
  setControllerGains();

  // ---------------------------------------------------------------------------
  // Compute errors
  // ---------------------------------------------------------------------------

  // Calculate errors
  vel_error_ = qdot_d_ - qdot_;
  error_ = q_d_ - q_;
  error_accum_ += error_;

  // Saturate integral error
  saturateIntegralError();

  // ---------------------------------------------------------------------------
  // compute control
  // ---------------------------------------------------------------------------

  Eigen::VectorXd tau(n_joints_);

  // Calculate joint torques according to PID control law
  tau = D * vel_error_ + I * error_accum_ + P * error_;

  // Publish desired joint torques
  publishTorques(tau);

  // Set desired torque to each joint
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau(i));
  }

  // filter joint positions
  q_d_ = filter_param_ * q_d_target_ + (1.0 - filter_param_) * q_d_;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \brief Calculates forward kinematics using the KDL library.
 */
bool JointPositionPIDSimController::fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf)
{
  if (q_in.size() != n_joints_)
    return false;

  KDL::ChainFkSolverPos_recursive solver(k_chain_);
  KDL::Frame end_effector_frame;
  KDL::JntArray joint_array(n_joints_);

  for (size_t i = 0; i < n_joints_; i++)
    joint_array(i) = q_in(i);

  // Obtain end-effector frame orientation and position within end_effector_frame variable
  solver.JntToCart(joint_array, end_effector_frame);

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      transf(i, j) = end_effector_frame(i, j);

  return true;
}

/**
 * \brief Updates the impedance gains using the dynamic reconfigure for simulation.
 *
 * After reading the gains from dynamic reconfigure, it updates the target PID gain matrices.
 */
void JointPositionPIDSimController::updateDynamicConfigGainsCallback(
    ::smalldrop_robot_arm::JointPositionPIDSimControllerConfig &config, uint32_t level)
{
  joint1_p_ = config.joint1_p;
  joint2_p_ = config.joint2_p;
  joint3_p_ = config.joint3_p;
  joint4_p_ = config.joint4_p;
  joint5_p_ = config.joint5_p;
  joint6_p_ = config.joint6_p;
  joint7_p_ = config.joint7_p;

  joint1_d_ = config.joint1_d;
  joint2_d_ = config.joint2_d;
  joint3_d_ = config.joint3_d;
  joint4_d_ = config.joint4_d;
  joint5_d_ = config.joint5_d;
  joint6_d_ = config.joint6_d;
  joint7_d_ = config.joint7_d;

  joint1_i_ = config.joint1_i;
  joint2_i_ = config.joint2_i;
  joint3_i_ = config.joint3_i;
  joint4_i_ = config.joint4_i;
  joint5_i_ = config.joint5_i;
  joint6_i_ = config.joint6_i;
  joint7_i_ = config.joint7_i;

  // Saturation limits
  i_clamp_ = config.i_clamp;

  updateDynamicConfigGains();
}

}  // namespace smalldrop_robot_arm

}  // namespace smalldrop

PLUGINLIB_EXPORT_CLASS(smalldrop::smalldrop_robot_arm::JointPositionPIDSimController,
                       controller_interface::ControllerBase)