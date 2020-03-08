// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

#include <smalldrop_robot_arm/cartesian_impedance_sim_controller.h>

namespace smalldrop_robot_arm
{

/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

bool CartesianImpedanceSimController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                  ros::NodeHandle &controller_nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!root_nh.getParam("/cartesian_impedance_sim_controller/arm_id", arm_id))
  {
    ROS_ERROR("CartesianImpedanceSimController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!root_nh.getParam("/cartesian_impedance_sim_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("CartesianImpedanceSimController: Invalid or no joint_names parameters "
              "provided, aborting controller init!");
    return false;
  }

  // Verify effort joint interface
  std::cout << "Verifying effort joint interface" << std::endl;
  auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceSimController: Error getting effort joint "
                     "interface from hardware");
    return false;
  }

  // Verify joint handles
  std::cout << "Verifying joint handles" << std::endl;
  for (size_t i = 0; i < 7; ++i)
  {
    try
    {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("CartesianImpedanceSimController: Exception getting joint handles: " << ex.what());
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
  dyn_config_gains_node_ = ros::NodeHandle("cartesian_impedance_controller/gains");
  dyn_config_gains_param_ = std::make_unique<dynamic_reconfigure::Server<smalldrop_robot_arm::CartesianImpedanceSimControllerConfig>>(dyn_config_gains_node_);
  dyn_config_gains_param_->setCallback(boost::bind(&CartesianImpedanceSimController::updateDynamicConfigGainsCallback, this, _1, _2));

  setupPublishersAndSubscribers(controller_nh);
  control_srv_ = controller_nh.advertiseService("send_torques_to_robot", &CartesianImpedanceSimController::sendTorquesToRobot, this);

  // ---------------------------------------------------------------------------
  // Init Values
  // ---------------------------------------------------------------------------
  T0ee_d_.setZero();
  T0ee_.setZero();
  cart_K.setZero();
  cart_D.setZero();
  cart_I.setZero();
  null_K.setZero();
  X0ee_d_.setZero();
  X0ee_d_prev_.setZero();
  R0ee_d_.setZero();
  orient_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  orient_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  error_.setZero();
  error_prev_.setZero();
  error_accum_.setZero();
  vel_d_.setZero();
  vel_error_.setZero();
  tau_.setZero();
  tau_initial_.setZero();
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
  nullspace_objective_.setZero();

  return true;
}

void CartesianImpedanceSimController::starting(const ros::Time &time)
{
  // Set desired end-effector position and euler angles
  Eigen::Matrix<double, 7, 1> qd;
  qd << 0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4;
  
  if (!fk(qd, T0ee_d_))
    ROS_ERROR("Cannot get forward kinematics.");

  Eigen::Affine3d transform(T0ee_d_);
  X0ee_d_ = transform.translation();
  R0ee_d_ = transform.rotation();

  X0ee_d_target_ = X0ee_d_;

  orient_d_ = Eigen::Quaterniond(transform.linear());
  orient_d_.normalize();
  orient_d_target_ = Eigen::Quaterniond(transform.linear());
  orient_d_target_.normalize();
}

void CartesianImpedanceSimController::update(const ros::Time &time, const ros::Duration &period)
{
  // Obtain joint positions (q), velocities (qdot_) and efforts (effort) from hardware interface
  for (size_t i = 0; i < n_joints_; i++)
  {
    q_(i) = joint_handles_[i].getPosition();
    qdot_(i) = joint_handles_[i].getVelocity();
    tau_(i) = joint_handles_[i].getEffort();

    if (time.toSec() <= 15) 
      tau_initial_ = tau_;
  }

  // Calculate X = f(q) using forward kinematics (FK)
  if (!fk(q_, T0ee_))
    ROS_ERROR("Cannot get forward kinematics.");
    
  Eigen::Affine3d transform(T0ee_);
  Eigen::Vector3d X0ee(transform.translation());
  Eigen::Matrix3d R0ee(transform.rotation());

  // Publish current pose
  publishCurrentPose(X0ee, R0ee);

  // ---------------------------------------------------------------------------
  // Calculate the dynamic parameters
  // ---------------------------------------------------------------------------

  // Calculate Jacobian
  if (!jacobian(J, q_))
    ROS_ERROR("Cannot calculate the jacobian.");

  // Calculate the dynamics: inertia, coriolis and gravity matrices
  if (!dynamic(q_, qdot_))
    ROS_ERROR("Cannot get the dynamics.");

  // Calculate Lambda which is the Mass Matrix of the task space
  // Λ(q) = (J * M(q)−1 * JT)−1
  Eigen::Matrix<double, 6, 6> lambda;
  lambda = (J * M.inverse() * J.transpose()).inverse();

  // Calculate the Dynamically consistent generalized inverse of the jacobian
  // J# = M(q)−1 * JT * Λ(q)
  Jhash = M.inverse() * J.transpose() * lambda;

  // ---------------------------------------------------------------------------
  // Set the controller gains
  // ---------------------------------------------------------------------------
  setControllerGains();

  // ---------------------------------------------------------------------------
  // compute error
  // ---------------------------------------------------------------------------

  // Calculate vel = J*qdot_ (velocity of end-effector)
  Eigen::Matrix<double, 6, 1> vel(J * qdot_);

  // Calculate position error
  error_.head(3) << X0ee_d_ - X0ee;

  // Calculate orientation error
  Eigen::Matrix3d Rcd(R0ee_d_ * R0ee.transpose());
  error_.tail(3) << R2r(Rcd);

  // Calculate velocity error
  vel_d_.head(3) <<  X0ee_d_ - X0ee_d_prev_; // desired velocity is derivative of desired position
  vel_error_ << vel_d_ - vel;
  X0ee_d_prev_ = X0ee_d_;

  // ---------------------------------------------------------------------------
  // compute control
  // ---------------------------------------------------------------------------

  // Declare variables
  // tau_task = JT*F
  // tau_null = (I - JT * J#T) * t0
  // tau_d = tau_task + tau_null 
  Eigen::VectorXd tau_task(7), tau_d(7), tau_null(7);

  // Calculate the null space objective function 
  nullSpaceObjectiveFunction(nullspace_objective_);
  Eigen::Matrix<double, 7, 1>  tau_o(M * null_K * nullspace_objective_);

  // Calculate nullspace torque (tau_null)
  tau_null << (Eigen::MatrixXd::Identity(7,7) - J.transpose() * Jhash.transpose()) * tau_o;

  // Calculate error accumulated
  calcIntegralError();

  // Saturate integral error
  saturateIntegralError();

  // Calculate task torque (tau_task)
  tau_task << J.transpose() * (cart_D * vel_error_ + cart_K * error_ + cart_I * error_accum_);

  error_prev_ = error_accum_;

  // Calculate final torque
  tau_d << tau_task + tau_null + C + g;

  // Publish torques, wrenches and tracking errors
  publishTorques(tau_d, tau_task, tau_null);
  publishWrenches();
  publishTrackingErrors();

  // Set desired torque to each joint
  if (!torques_to_robot_) 
    tau_d << 0, 0, 0, 0, 0, 0, 0;

  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_d(i));

  // filter position
  X0ee_d_ = filter_param_ * X0ee_d_target_ + (1.0 - filter_param_) * X0ee_d_;
  // filter orientation
  orient_d_ = orient_d_.slerp(filter_param_, orient_d_target_);
  orient_d_.normalize();
  R0ee_d_ = orient_d_.toRotationMatrix();
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \brief Calculate the Jacobian matrix using KDL library.
 */
bool CartesianImpedanceSimController::calcJacobian(KDL::Jacobian &Jac, const Eigen::Matrix<double, 7, 1> &q_in)
{
  if (q_in.size() != n_joints_) return false;

  KDL::ChainJntToJacSolver solver(k_chain_);
  KDL::JntArray joint_array(n_joints_);
  
  for (size_t i = 0; i < n_joints_; i++)
    joint_array(i) = q_in(i);

  // Obtain jacobian
  solver.JntToJac(joint_array, Jac);

  return true;
}

/**
 * \brief Transforms the KDL format Jacobian to an Eigen matrix.
 */ 
bool CartesianImpedanceSimController::jacobian(Eigen::Matrix<double, 6, 7> &J_out, const Eigen::Matrix<double, 7, 1> &q_in)
{
  KDL::Jacobian Jac(n_joints_);

  if (calcJacobian(Jac, q_in))
  {
    unsigned int rows = Jac.rows();
    unsigned int columns = Jac.columns();

    for (size_t row = 0; row < rows; row++)
      for (size_t col = 0; col < columns; col++)
        J_out(row, col) = Jac(row, col);

    return true;
  }
  return false;
}

/**
 * \brief Calculates forward kinematics using the KDL library.
 */
bool CartesianImpedanceSimController::fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf)
{
  if (q_in.size() != n_joints_) return false;

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
 * \brief Calculate robot dynamics using KDL library. The dynamic class members are updated inside the method.
 */ 
bool CartesianImpedanceSimController::dynamic(Eigen::Matrix<double, 7, 1>& q, Eigen::Matrix<double, 7, 1>& qdot_)
{
  if(q.size() != n_joints_ || qdot_.size() != n_joints_){
    return false;
  }

  KDL::Vector g_vector = {0.0, 0.0, -9.80665};   // Gravity vector
  KDL::ChainDynParam chain_dyn(k_chain_, g_vector);
  
  KDL::JntArray q_jarray(n_joints_);
  KDL::JntArray qdot_jarray(n_joints_);

  KDL::JntSpaceInertiaMatrix kdl_inertia;
  KDL::JntArray kdl_coriolis;
  KDL::JntArray kdl_gravity;

  // define inertia matrix with correct size (rows and columns)
  kdl_inertia = KDL::JntSpaceInertiaMatrix(n_joints_);
  kdl_coriolis = KDL::JntArray(n_joints_);
  kdl_gravity = KDL::JntArray(n_joints_);

  for (size_t i = 0; i < n_joints_; i++)
  {
    q_jarray(i) = q(i);
    qdot_jarray(i) = qdot_(i);
  }

  // Get Inertia Matrix
  if (chain_dyn.JntToMass(q_jarray, kdl_inertia) != KDL::SolverI::E_NOERROR)
    return false;

  // Get Coriolis Vector
  if (chain_dyn.JntToCoriolis(q_jarray, qdot_jarray, kdl_coriolis) != KDL::SolverI::E_NOERROR)
    return false;

  // Get Gravity Matrix
  if (chain_dyn.JntToGravity(q_jarray, kdl_gravity) != KDL::SolverI::E_NOERROR)
    return false;

  for (size_t i = 0; i < n_joints_; i++){
    for (size_t j = 0; j < n_joints_; j++){
      M(i,j) = kdl_inertia(i,j);
      C(i) = kdl_coriolis(i);
      g(i) = kdl_gravity(i);
    }
  }

  return true;
}

/**
 * \brief Updates the impedance gains using the dynamic reconfigure for simulation.
 * 
 * After reading the gains from dynamic reconfigure, it updates the target impedance gain matrices 
 * and nullspace stiffness matrix.
 */
void CartesianImpedanceSimController::updateDynamicConfigGainsCallback(smalldrop_robot_arm::CartesianImpedanceSimControllerConfig &config, uint32_t level)
{
  Kpx = config.Kpx;
  Kpy = config.Kpy;
  Kpz = config.Kpz;
  Kox = config.Kox;
  Koy = config.Koy;
  Koz = config.Koz;

  Dpx = config.Dpx;
  Dpy = config.Dpy;
  Dpz = config.Dpz;
  Dox = config.Dox;
  Doy = config.Doy;
  Doz = config.Doz;

  Ipx = config.Ipx;
  Ipy = config.Ipy;
  Ipz = config.Ipz;
  Iox = config.Iox;
  Ioy = config.Ioy;
  Ioz = config.Ioz;

  Kpn = config.Kpn;

  // Saturation limits
  i_clamp_p_ = config.Ip_clamp;
  i_clamp_o_ = config.Io_clamp;

  updateDynamicConfigGains();
}

/**
 * \brief Service callback to set if calculated torques should be sent to the robot.
 */
bool CartesianImpedanceSimController::sendTorquesToRobot(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  torques_to_robot_ = req.data;
  res.success = true;
  return true;
}

}  // namespace smalldrop_robot_arm

PLUGINLIB_EXPORT_CLASS(smalldrop_robot_arm::CartesianImpedanceSimController, controller_interface::ControllerBase)