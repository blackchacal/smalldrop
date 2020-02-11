#include "panda_3dbioprint_real/gravity_compensation_controller.h"

namespace panda_3dbioprint_real
{

bool GravityCompensationController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!nh.getParam("/cartesian_impedance_controller/arm_id", arm_id))
  {
    ROS_ERROR("GravityCompensationController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!nh.getParam("/cartesian_impedance_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("GravityCompensationController: Invalid or no joint_names parameters "
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
    model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
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
    state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
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
    ROS_ERROR_STREAM("GravityCompensationController: Error getting effort joint "
                     "interface from hardware");
    return false;
  }

  // Verify joint handles
  std::cout << "Verifying joint handles" << std::endl;
  for (size_t i = 0; i < 7; ++i)
  {
    try
    {
      joint_handles.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM("GravityCompensationController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Load joint model from urdf to get joint limits
  urdf::Model model;
  if (model.initParam("/robot_description"))
  {
    robot_joints = model.joints_;
  }
  else
  {
    ROS_ERROR("Unable to read the robot model from URDF.");
  }

  posePub = nh.advertise<geometry_msgs::Pose>("/cartesian_impedance_controller/current_pose", 10);
  wrenchPub = nh.advertise<geometry_msgs::Wrench>("/cartesian_impedance_controller/wrench", 10);

  // ---------------------------------------------------------------------------
  // Init Values
  // ---------------------------------------------------------------------------
  T0ee.setZero();
  maxJointLimits << robot_joints[joint_handles[0].getName()].get()->limits.get()->upper,
                    robot_joints[joint_handles[1].getName()].get()->limits.get()->upper,
                    robot_joints[joint_handles[2].getName()].get()->limits.get()->upper,
                    robot_joints[joint_handles[3].getName()].get()->limits.get()->upper,
                    robot_joints[joint_handles[4].getName()].get()->limits.get()->upper,
                    robot_joints[joint_handles[5].getName()].get()->limits.get()->upper,
                    robot_joints[joint_handles[6].getName()].get()->limits.get()->upper;
  minJointLimits << robot_joints[joint_handles[0].getName()].get()->limits.get()->lower,
                    robot_joints[joint_handles[1].getName()].get()->limits.get()->lower,
                    robot_joints[joint_handles[2].getName()].get()->limits.get()->lower,
                    robot_joints[joint_handles[3].getName()].get()->limits.get()->lower,
                    robot_joints[joint_handles[4].getName()].get()->limits.get()->lower,
                    robot_joints[joint_handles[5].getName()].get()->limits.get()->lower,
                    robot_joints[joint_handles[6].getName()].get()->limits.get()->lower;
  return true;
}

void GravityCompensationController::starting(const ros::Time &time)
{
  // We want to start the controller by setting as the desired position 
  // the actual position. This avoids problems associated with sending the robot
  // to a specified position right away. That can cause bounces and instabillities.

  // Read the robot state
  franka::RobotState initial_state = state_handle->getRobotState();

  // Get transformation from end-effector to base T0ee_d
  T0ee = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());

  Eigen::Affine3d transform(T0ee);
  X0ee = transform.translation();
  R0ee = transform.rotation();
}

void GravityCompensationController::update(const ros::Time &time, const ros::Duration &period)
{
  // Get robot state
  franka::RobotState robot_state = state_handle->getRobotState();

  // Obtain joint positions (q), velocities (qdot) and efforts (effort) from hardware interface
  q = Eigen::Matrix<double, 7, 1>::Map(robot_state.q.data());
  qdot = Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data());

  // Desired link-side joint torque sensor signals without gravity. Unit: [Nm]
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // Calculate X = f(q) using forward kinematics (FK)
  T0ee = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
    
  Eigen::Affine3d transform(T0ee);
  Eigen::Vector3d X0ee(transform.translation());
  Eigen::Matrix3d R0ee(transform.rotation());

  // Publish current pose
  geometry_msgs::Pose msg;
  msg.position.x = X0ee[0];
  msg.position.y = X0ee[1];
  msg.position.z = X0ee[2];
  Eigen::Quaterniond quat(R0ee);
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();
  posePub.publish(msg);

  // Calculate Jacobian
  std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);
  J = Eigen::Matrix<double, 6, 7>::Map(jacobian_array.data());

  // Calculate the dynamics: inertia, coriolis and gravity matrices
  M = Eigen::Matrix<double, 7, 7>::Map(model_handle->getMass().data());
  C = Eigen::Matrix<double, 7, 1>::Map(model_handle->getCoriolis().data());
  g = Eigen::Matrix<double, 7, 1>::Map(model_handle->getGravity().data());

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

  // Publish wrenchs
  geometry_msgs::Wrench wrench_msg;

  // Prepare force msg
  franka::RobotState state = state_handle->getRobotState();
  std::array<double, 6> wrench = state.O_F_ext_hat_K;
  wrench_msg.force.x = wrench[0]; 
  wrench_msg.force.y = wrench[1]; 
  wrench_msg.force.z = wrench[2];
  wrench_msg.torque.x = wrench[3]; 
  wrench_msg.torque.y = wrench[4]; 
  wrench_msg.torque.z = wrench[5]; 

  // Publish the messages
  wrenchPub.publish(wrench_msg);

  // Set desired torque to each joint
  for (size_t i = 0; i < 7; ++i)
  {
    joint_handles[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> GravityCompensationController::saturateTorqueRate(
  const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  const Eigen::Matrix<double, 7, 1>& tau_J_d) 
{
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max);
  }
  return tau_d_saturated;
}

} // namespace panda_3dbioprint_real

PLUGINLIB_EXPORT_CLASS(panda_3dbioprint_real::GravityCompensationController, controller_interface::ControllerBase)