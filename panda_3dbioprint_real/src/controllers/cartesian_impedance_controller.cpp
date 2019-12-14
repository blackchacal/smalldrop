#include "panda_3dbioprint_real/cartesian_impedance_controller.h"
#include <ros/package.h>

namespace panda_3dbioprint_real
{

bool CartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!nh.getParam("/cartesian_impedance_controller/arm_id", arm_id))
  {
    ROS_ERROR("CartesianImpedanceController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!nh.getParam("/cartesian_impedance_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("CartesianImpedanceController: Invalid or no joint_names parameters "
              "provided, aborting controller init!");
    return false;
  }

  // Verify robot model interface
  std::cout << "Verifying robot model interface" << std::endl;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // Verify robot state interface
  std::cout << "Verifying robot state interface" << std::endl;
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // Verify effort joint interface
  std::cout << "Verifying effort joint interface" << std::endl;
  auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianImpedanceController: Error getting effort joint "
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
      ROS_ERROR_STREAM("CartesianImpedanceController: Exception getting joint handles: " << ex.what());
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

  // Setup dynamic reconfigure
  dyn_config_gains_node = ros::NodeHandle("cartesian_impedance_controller/gains");
  dyn_config_gains_param = std::make_unique<dynamic_reconfigure::Server<panda_3dbioprint_real::CartesianImpedanceControllerConfig>>(dyn_config_gains_node);
  dyn_config_gains_param->setCallback(boost::bind(&CartesianImpedanceController::DesiredGainsParamCallback, this, _1, _2));

  poseSub = nh.subscribe("/cartesian_impedance_controller/desired_pose", 10, &CartesianImpedanceController::updatePoseCallback, this);
  posePub = nh.advertise<geometry_msgs::Pose>("/cartesian_impedance_controller/current_pose", 10);

  // ---------------------------------------------------------------------------
  // Init Values
  // ---------------------------------------------------------------------------
  T0ee_d.setZero();
  T0ee.setZero();
  cart_K.setZero();
  cart_D.setZero();
  null_K.setZero();
  X_d.setZero();
  R_d.setZero();
  error.setZero();
  vel_d.setZero();
  vel_error.setZero();
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
  gradient.setZero();

  return true;
}

void CartesianImpedanceController::starting(const ros::Time &time)
{
  // We want to start the controller by setting as the desired position 
  // the actual position. This avoids problems associated with sending the robot
  // to a specified position right away. That can cause bounces and instabillities.

  // Read the robot state
  franka::RobotState initial_state = state_handle->getRobotState();

  // Get transformation from end-effector to base T0ee_d
  T0ee_d = Eigen::Matrix4d::Map(initial_state.O_T_EE.data());

  Eigen::Affine3d transform(T0ee_d);
  X_d = transform.translation();
  R_d = transform.rotation();

  X_d_target = X_d;
  orient_d_target = R_d;
}

void CartesianImpedanceController::update(const ros::Time &time, const ros::Duration &period)
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
  Eigen::Vector3d X(transform.translation());
  Eigen::Matrix3d R(transform.rotation());

  // Publish current pose
  geometry_msgs::Pose msg;
  msg.position.x = X[0];
  msg.position.y = X[1];
  msg.position.z = X[2];
  Eigen::Quaterniond quat(R);
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
  // Set the controller gains
  // ---------------------------------------------------------------------------
  
  // Filter the gains so that the response is not abrupt!
  Kp_d = filter_param * Kp_d_target + (1.0 - filter_param) * Kp_d;
  Dp_d = filter_param * Dp_d_target + (1.0 - filter_param) * Dp_d;
  Ko_d = filter_param * Ko_d_target + (1.0 - filter_param) * Ko_d;
  Do_d = filter_param * Do_d_target + (1.0 - filter_param) * Do_d;

  // Change from end-effector frame to base frame
  Eigen::Matrix3d Kp(R_d * Kp_d * R_d.transpose());  // cartesian position stiffness
  Eigen::Matrix3d Dp(R_d * Dp_d * R_d.transpose());  // cartesian position damping
  Eigen::Matrix3d Ko(R_d * Ko_d * R_d.transpose());  // cartesian orientation stiffness
  Eigen::Matrix3d Do(R_d * Do_d * R_d.transpose());  // cartesian orientation damping

  cart_K.setIdentity();
  cart_K.topLeftCorner(3, 3) << Kp;
  cart_K.bottomRightCorner(3, 3) << Ko;

  cart_D.setIdentity();
  cart_D.topLeftCorner(3, 3) << Dp;
  cart_D.bottomRightCorner(3, 3) << Do;

  // ---------------------------------------------------------------------------
  // compute error
  // ---------------------------------------------------------------------------
    
  // Get position and orientation from topic
  X_d = X_d_target;
  R_d = orient_d_target.toRotationMatrix();
  // Calculate vel = J*qdot (velocity of end-effector)
  Eigen::Matrix<double, 6, 1> vel(J * qdot);

  // Calculate position error
  error.head(3) << X_d - X;

  // Calculate orientation error
  Eigen::Matrix3d Rcd(R_d * R.transpose());
  error.tail(3) << R2r(Rcd);

  // Calculate velocity error
  vel_error << vel_d - vel;

  // ---------------------------------------------------------------------------
  // compute control
  // ---------------------------------------------------------------------------

  // Declare variables
  // tau_task = JT*F
  // tau_null = (I - JT * J#T) * t0
  // tau_d = tau_task + tau_null 
  Eigen::VectorXd tau_task(7), tau_d(7), tau_null(7);

  // The performance optimization torque
  gradient_mechanical_joint_limit<7>(gradient, q, maxJointLimits, minJointLimits);
  Eigen::Matrix<double, 7, 1>  tau_o( M * null_K * gradient );

  // Calculate nullspace torque (tau_null)
  tau_null << (Eigen::MatrixXd::Identity(7,7) - J.transpose() * Jhash.transpose()) * tau_o;

  // Calculate task torque (tau_task)
  tau_task << J.transpose() * ( cart_D * vel_error + cart_K * error);

  // Calculate final torque
  tau_d << tau_task + tau_null + C;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // Set desired torque to each joint
  for (size_t i = 0; i < 7; ++i)
  {
    joint_handles[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
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

Eigen::Vector3d CartesianImpedanceController::R2r(const Eigen::Matrix3d &Rotation)
{
  Eigen::Vector3d rotation_vector, aux;
  aux << Rotation(2, 1) - Rotation(1, 2), Rotation(0, 2) - Rotation(2, 0), Rotation(1, 0) - Rotation(0, 1);
  rotation_vector = 0.5 * aux;

  return rotation_vector;
}

void CartesianImpedanceController::DesiredGainsParamCallback(panda_3dbioprint_real::CartesianImpedanceControllerConfig &config, uint32_t level)
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
  Kpn = config.Kpn;

  // position stiffness in desired frame
  Kp_d_target << Kpx,   0,   0,
            0, Kpy,   0,
            0,   0, Kpz;

  // orientation stiffness in desired frame
  Ko_d_target << Kox,   0,   0,
            0, Koy,   0,
            0,   0, Koz;

  // position damping in desired frame
  Dp_d_target << Dpx,   0,   0,
            0, Dpy,   0,
            0,   0, Dpz;

  // orientation damping in desired frame
  Do_d_target << Dox,   0,   0,
            0, Doy,   0,
            0,   0, Doz;

  // nullspace Gains
  null_K = Kpn * null_K.setIdentity();
}

void CartesianImpedanceController::updatePoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
  X_d_target << msg->position.x, msg->position.y, msg->position.z;

  Eigen::Quaterniond last_orient_d_target(orient_d_target);
  orient_d_target.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
  if (last_orient_d_target.coeffs().dot(orient_d_target.coeffs()) < 0.0)
    orient_d_target.coeffs() << -orient_d_target.coeffs();
}

double CartesianImpedanceController::derivative_computation( const double q_i, const double maxJointLimit_i, const double minJointLimit_i){
  double result;
  double average_joint;
  average_joint = ( maxJointLimit_i + minJointLimit_i) / 2.0;
  result = - ( ( ( q_i - average_joint ) / pow( ( maxJointLimit_i - minJointLimit_i ), 2 ) ) );

  return result;
}

template<int N> // number of joints or DOF
void CartesianImpedanceController::gradient_mechanical_joint_limit( Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, const Eigen::Matrix<double, N, 1> q, const Eigen::Matrix<double, N, 1> maxJointLimits, const Eigen::Matrix<double, N, 1> minJointLimits ){
  for ( int i = 0; i < q.rows(); i++ ){
    gradient_mechanical_joint_limit_out(i) = derivative_computation( q(i), maxJointLimits(i), minJointLimits(i) );
  }
}

} // namespace panda_3dbioprint_real

PLUGINLIB_EXPORT_CLASS(panda_3dbioprint_real::CartesianImpedanceController, controller_interface::ControllerBase)