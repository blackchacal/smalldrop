// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

#include "smalldrop_robot_arm/gravity_compensation_real_controller.h"

namespace smalldrop_robot_arm
{

/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

bool GravityCompensationRealController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!nh.getParam("/gravity_compensation_real_controller/arm_id", arm_id))
  {
    ROS_ERROR("GravityCompensationRealController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!nh.getParam("/gravity_compensation_real_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("GravityCompensationRealController: Invalid or no joint_names parameters "
              "provided, aborting controller init!");
    return false;
  }

  // Verify robot model interface
  std::cout << "Verifying robot model interface" << std::endl;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "GravityCompensationExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "GravityCompensationExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // Verify robot state interface
  std::cout << "Verifying robot state interface" << std::endl;
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "GravityCompensationExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "GravityCompensationExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // Verify effort joint interface
  std::cout << "Verifying effort joint interface" << std::endl;
  auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM("GravityCompensationRealController: Error getting effort joint "
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
      ROS_ERROR_STREAM("GravityCompensationRealController: Exception getting joint handles: " << ex.what());
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

  setupPublishersAndSubscribers(nh);

  // ---------------------------------------------------------------------------
  // Init Values
  // ---------------------------------------------------------------------------
  T0ee_.setZero();
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
  return true;
}

void GravityCompensationRealController::starting(const ros::Time &time)
{
  // We want to start the controller by setting as the desired position 
  // the actual position. This avoids problems associated with sending the robot
  // to a specified position right away. That can cause bounces and instabillities.

  // Read the robot state
  franka::RobotState initial_state = state_handle_->getRobotState();

  // Get transformation from end-effector to base T0ee_d
  T0ee_ = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());

  Eigen::Affine3d transform(T0ee_);
  Eigen::Vector3d X0ee_ = transform.translation();
  Eigen::Matrix3d R0ee_ = transform.rotation();
}

void GravityCompensationRealController::update(const ros::Time &time, const ros::Duration &period)
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
  Eigen::Vector3d X0ee_(transform.translation());
  Eigen::Matrix3d R0ee_(transform.rotation());

  // Publish current pose
  publishCurrentPose(X0ee_, R0ee_);

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
  // compute control
  // ---------------------------------------------------------------------------

  // Declare variables
  // tau_d = [0, 0, 0, 0, 0, 0, 0]
  Eigen::VectorXd tau_d(7);

  // Calculate final torque
  tau_d << 0, 0, 0, 0, 0, 0, 0;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // Publish wrenches and tracking errors
  publishWrenches();
  publishTrackingErrors();

  // Set desired torque to each joint
  for (size_t i = 0; i < 7; ++i)
    joint_handles_[i].setCommand(tau_d(i));
}

/**
 * \brief Clamps the joint torques if they go beyond the defined limits.
 */
Eigen::Matrix<double, 7, 1> GravityCompensationRealController::saturateTorqueRate(
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
 * \brief Publishes the robot external wrenches to a topic.
 */
void GravityCompensationRealController::publishWrenches(void)
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

} // namespace smalldrop_robot_arm

PLUGINLIB_EXPORT_CLASS(smalldrop_robot_arm::GravityCompensationRealController, controller_interface::ControllerBase)