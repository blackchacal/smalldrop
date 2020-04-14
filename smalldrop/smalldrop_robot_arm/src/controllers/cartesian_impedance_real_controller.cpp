// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

#include "smalldrop_robot_arm/cartesian_impedance_real_controller.h"

namespace smalldrop
{
namespace smalldrop_robot_arm
{
/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

bool CartesianImpedanceRealController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!nh.getParam("/cartesian_impedance_real_controller/arm_id", arm_id))
  {
    ROS_ERROR("CartesianImpedanceRealController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!nh.getParam("/cartesian_impedance_real_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("CartesianImpedanceRealController: Invalid or no joint_names parameters "
              "provided, aborting controller init!");
    return false;
  }

  // Verify robot model interface
  std::cout << "Verifying robot model interface" << std::endl;
  auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try
  {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException &ex)
  {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // Verify robot state interface
  std::cout << "Verifying robot state interface" << std::endl;
  auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try
  {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  }
  catch (hardware_interface::HardwareInterfaceException &ex)
  {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  // Verify effort joint interface
  std::cout << "Verifying effort joint interface" << std::endl;
  auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceRealController: Error getting effort joint "
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
      ROS_ERROR_STREAM("CartesianImpedanceRealController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

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
  dyn_config_gains_param_ = std::make_unique<
      dynamic_reconfigure::Server<::smalldrop_robot_arm::CartesianImpedanceRealControllerConfig>>(
      dyn_config_gains_node_);
  dyn_config_gains_param_->setCallback(
      boost::bind(&CartesianImpedanceRealController::updateDynamicConfigGainsCallback, this, _1, _2));

  setupPublishersAndSubscribers(nh);

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

  // Workspace limits
  std::vector<double> x_limits;
  std::vector<double> x_limits_defaults = {0.1, 0.5};
  std::vector<double> y_limits;
  std::vector<double> y_limits_defaults = {-0.2, 0.2};
  std::vector<double> z_limits;
  std::vector<double> z_limits_defaults = {0, 0.5};
  nh.param("smalldrop/robot/workspace/x_limits", x_limits, x_limits_defaults);
  nh.param("smalldrop/robot/workspace/y_limits", y_limits, y_limits_defaults);
  nh.param("smalldrop/robot/workspace/z_limits", z_limits, z_limits_defaults);

  wsp_x_min_limit_ = x_limits[0];
  wsp_x_max_limit_ = x_limits[1];
  wsp_y_min_limit_ = y_limits[0];
  wsp_y_max_limit_ = y_limits[1];
  wsp_z_min_limit_ = z_limits[0];
  wsp_z_max_limit_ = z_limits[1];

  return true;
}

void CartesianImpedanceRealController::starting(const ros::Time &time)
{
  // We want to start the controller by setting as the desired position
  // the actual position. This avoids problems associated with sending the robot
  // to a specified position right away. That can cause bounces and instabillities.

  // Read the robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // Get transformation from end-effector to base T0ee_d
  T0ee_d_ = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());

  Eigen::Affine3d transform(T0ee_d_);
  X0ee_d_ = transform.translation();
  R0ee_d_ = transform.rotation();

  X0ee_d_target_ = X0ee_d_;

  orient_d_ = Eigen::Quaterniond(transform.linear());
  orient_d_.normalize();
  orient_d_target_ = Eigen::Quaterniond(transform.linear());
  orient_d_target_.normalize();
}

void CartesianImpedanceRealController::update(const ros::Time &time, const ros::Duration &period)
{
  // Get robot state
  franka::RobotState robot_state = state_handle_->getRobotState();

  // Obtain joint positions (q), velocities (qdot) and efforts (effort) from hardware interface
  q_ = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
  qdot_ = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());

  // Desired link-side joint torque sensor signals without gravity. Unit: [Nm]
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // Calculate X = f(q) using forward kinematics (FK)
  T0ee_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());

  Eigen::Affine3d transform(T0ee_);
  Eigen::Vector3d X0ee(transform.translation());
  Eigen::Matrix3d R0ee(transform.rotation());

  // Publish current pose
  publishCurrentPose(X0ee, R0ee);

  // ---------------------------------------------------------------------------
  // Calculate the dynamic parameters
  // ---------------------------------------------------------------------------

  // Calculate Jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  J = Eigen::Matrix<double, 6, 7>::Map(jacobian_array.data());

  // Calculate the dynamics: inertia, coriolis and gravity matrices
  M = Eigen::Matrix<double, 7, 7>::Map(model_handle_->getMass().data());
  C = Eigen::Matrix<double, 7, 1>::Map(model_handle_->getCoriolis().data());
  g = Eigen::Matrix<double, 7, 1>::Map(model_handle_->getGravity().data());

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

  // Calculate vel = J*qdot (velocity of end-effector)
  Eigen::Matrix<double, 6, 1> vel(J * qdot_);

  // Calculate position error
  error_.head(3) << X0ee_d_ - X0ee;

  // Calculate orientation error
  Eigen::Matrix3d Rcd(R0ee_d_ * R0ee.transpose());
  error_.tail(3) << R2r(Rcd);

  // Calculate velocity error
  vel_d_.head(3) << X0ee_d_ - X0ee_d_prev_;  // desired velocity is derivative of desired position
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
  Eigen::Matrix<double, 7, 1> tau_o(M * null_K * nullspace_objective_);

  // Calculate nullspace torque (tau_null)
  tau_null << (Eigen::MatrixXd::Identity(7, 7) - J.transpose() * Jhash.transpose()) * tau_o;

  // Calculate error accumulated
  calcIntegralError();

  // Saturate integral error
  saturateIntegralError();

  // Calculate task torque (tau_task)
  tau_task << J.transpose() * (cart_D * vel_error_ + cart_K * error_ + cart_I * error_accum_);

  error_prev_ = error_accum_;

  // Calculate final torque
  tau_d << tau_task + tau_null + C;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // Publish torques, wrenches and tracking errors
  publishTorques(tau_d, tau_task, tau_null);
  publishWrenches();
  publishTrackingErrors();

  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_d(i));

  // filter position
  X0ee_d_ = filter_param_ * X0ee_d_target_ + (1.0 - filter_param_) * X0ee_d_;
  // filter orientation
  orient_d_ = orient_d_.slerp(filter_param_, orient_d_target_);
  orient_d_.normalize();
  R0ee_d_ = orient_d_.toRotationMatrix();
}

/**
 * \brief Clamps the joint torques if they go beyond the defined limits.
 */
Eigen::Matrix<double, 7, 1> CartesianImpedanceRealController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1> &tau_d, const Eigen::Matrix<double, 7, 1> &tau_J_d)
{
  Eigen::Matrix<double, 7, 1> tau_d_saturated;

  for (size_t i = 0; i < 7; i++)
  {
    double difference = tau_d[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

/**
 * \brief Updates the impedance gains using the dynamic reconfigure for real robot operation.
 *
 * After reading the gains from dynamic reconfigure, it updates the target impedance gain matrices
 * and nullspace stiffness matrix.
 */
void CartesianImpedanceRealController::updateDynamicConfigGainsCallback(
    ::smalldrop_robot_arm::CartesianImpedanceRealControllerConfig &config, uint32_t level)
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
 * \brief Publishes the robot external wrenches to a topic.
 */
void CartesianImpedanceRealController::publishWrenches(void)
{
  geometry_msgs::Wrench wrench_msg;

  // Prepare force msg
  franka::RobotState state = state_handle_->getRobotState();
  std::array<double, 6> wrench = state.O_F_ext_hat_K;
  wrench_msg.force.x = wrench[0];
  wrench_msg.force.y = wrench[1];
  wrench_msg.force.z = wrench[2];
  wrench_msg.torque.x = wrench[3];
  wrench_msg.torque.y = wrench[4];
  wrench_msg.torque.z = wrench[5];

  wrench_pub_.publish(wrench_msg);
}

}  // namespace smalldrop_robot_arm
}  // namespace smalldrop

PLUGINLIB_EXPORT_CLASS(smalldrop::smalldrop_robot_arm::CartesianImpedanceRealController,
                       controller_interface::ControllerBase)