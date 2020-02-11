#include <panda_3dbioprint_simulation/cartesian_impedance_controller.h>
#include <ros/package.h>

namespace panda_3dbioprint_simulation
{
bool CartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                  ros::NodeHandle &controller_nh)
{
  // Verify robot
  std::cout << "Verifying robot id" << std::endl;
  std::string arm_id;
  if (!root_nh.getParam("/cartesian_impedance_controller/arm_id", arm_id))
  {
    ROS_ERROR("CartesianImpedanceController: Could not read the parameter 'arm_id'.");
    return false;
  }

  // Verify number of joints
  std::cout << "Verifying number of joints" << std::endl;
  std::vector<std::string> joint_names;
  if (!root_nh.getParam("/cartesian_impedance_controller/joints", joint_names) || joint_names.size() != 7)
  {
    ROS_ERROR("CartesianImpedanceController: Invalid or no joint_names parameters "
              "provided, aborting controller init!");
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

  // Parse KDL tree
  std::string robot_desc_string;
  root_nh.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, k_tree))
  {
    ROS_ERROR("Failed to construct kdl tree.");
    return false;
  }

  // Get chain from kdl tree
  if (!k_tree.getChain("panda_link0", "panda_EE", k_chain))
  {
    ROS_ERROR("Failed to get chain from kdl tree!");
  }

  n_joints = k_chain.getNrOfJoints();

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
  dyn_config_pid_gains_node = ros::NodeHandle("cartesian_impedance_controller/gains");
  dyn_config_pid_gains_param = std::make_unique<dynamic_reconfigure::Server<panda_3dbioprint_simulation::CartesianImpedanceControllerConfig>>(dyn_config_pid_gains_node);
  dyn_config_pid_gains_param->setCallback(boost::bind(&CartesianImpedanceController::DesiredGainsParamCallback, this, _1, _2));

  poseSub = controller_nh.subscribe("/cartesian_impedance_controller/desired_pose", 10, &CartesianImpedanceController::updatePoseCallback, this);
  posePub = controller_nh.advertise<geometry_msgs::Pose>("/cartesian_impedance_controller/current_pose", 10);
  tauPub = controller_nh.advertise<panda_3dbioprint_simulation::Tau>("/cartesian_impedance_controller/tau", 10);
  wrenchPub = controller_nh.advertise<geometry_msgs::Wrench>("/cartesian_impedance_controller/wrench", 10);
  errorPub = controller_nh.advertise<panda_3dbioprint_simulation::TrackingError>("/cartesian_impedance_controller/error", 10);
  controlSrv = controller_nh.advertiseService("send_torques_to_robot", &CartesianImpedanceController::sendTorquesToRobot, this);

  // ---------------------------------------------------------------------------
  // Init Values
  // ---------------------------------------------------------------------------
  T0ee_d.setZero();
  T0ee.setZero();
  cart_K.setZero();
  cart_D.setZero();
  cart_I.setZero();
  null_K.setZero();
  X0ee_d.setZero();
  X0ee_d_prev.setZero();
  R0ee_d.setZero();
  orient_d.coeffs() << 0.0, 0.0, 0.0, 1.0;
  orient_d_target.coeffs() << 0.0, 0.0, 0.0, 1.0;
  error.setZero();
  error_prev.setZero();
  error_accum.setZero();
  vel_d.setZero();
  vel_error.setZero();
  tau.setZero();
  tau_initial.setZero();
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
  // Set desired end-effector position and euler angles
  Eigen::Matrix<double, 7, 1> qd;
  qd << 0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4;
  
  if (!fk(qd, T0ee_d))
    ROS_ERROR("Cannot get forward kinematics.");

  Eigen::Affine3d transform(T0ee_d);
  X0ee_d = transform.translation();
  R0ee_d = transform.rotation();

  X0ee_d_target = X0ee_d;

  orient_d = Eigen::Quaterniond(transform.linear());
  orient_d.normalize();
  orient_d_target = Eigen::Quaterniond(transform.linear());
  orient_d_target.normalize();
}

void CartesianImpedanceController::update(const ros::Time &time, const ros::Duration &period)
{
  // Obtain joint positions (q), velocities (qdot) and efforts (effort) from hardware interface
  for (size_t i = 0; i < n_joints; i++)
  {
    q(i) = joint_handles[i].getPosition();
    qdot(i) = joint_handles[i].getVelocity();
    tau(i) = joint_handles[i].getEffort();

    if (time.toSec() <= 15) 
      tau_initial = tau;
  }

  // Calculate X = f(q) using forward kinematics (FK)
  if (!fk(q, T0ee))
    ROS_ERROR("Cannot get forward kinematics.");
    
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
  if (!jacobian(J, q))
    ROS_ERROR("Cannot calculate the jacobian.");

  // Calculate the dynamics: inertia, coriolis and gravity matrices
  if (!dynamic(q, qdot))
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
  
  // Filter the gains so that the response is not abrupt!
  Kp_d = filter_param * Kp_d_target + (1.0 - filter_param) * Kp_d;
  Dp_d = filter_param * Dp_d_target + (1.0 - filter_param) * Dp_d;
  Ko_d = filter_param * Ko_d_target + (1.0 - filter_param) * Ko_d;
  Do_d = filter_param * Do_d_target + (1.0 - filter_param) * Do_d;
  Ip_d = filter_param * Ip_d_target + (1.0 - filter_param) * Ip_d;
  Io_d = filter_param * Io_d_target + (1.0 - filter_param) * Io_d;
  null_K_d = filter_param * null_K_d_target + (1.0 - filter_param) * null_K_d;

  // Change from end-effector frame to base frame
  Eigen::Matrix3d Kp(R0ee_d * Kp_d * R0ee_d.transpose());  // cartesian position stiffness
  Eigen::Matrix3d Dp(R0ee_d * Dp_d * R0ee_d.transpose());  // cartesian position damping
  Eigen::Matrix3d Ip(R0ee_d * Ip_d * R0ee_d.transpose());  // cartesian position integral
  Eigen::Matrix3d Ko(R0ee_d * Ko_d * R0ee_d.transpose());  // cartesian orientation stiffness
  Eigen::Matrix3d Do(R0ee_d * Do_d * R0ee_d.transpose());  // cartesian orientation damping
  Eigen::Matrix3d Io(R0ee_d * Io_d * R0ee_d.transpose());  // cartesian orientation integral

  cart_K.setIdentity();
  cart_K.topLeftCorner(3, 3) << Kp;
  cart_K.bottomRightCorner(3, 3) << Ko;

  cart_D.setIdentity();
  cart_D.topLeftCorner(3, 3) << Dp;
  cart_D.bottomRightCorner(3, 3) << Do;

  cart_I.setIdentity();
  cart_I.topLeftCorner(3, 3) << Ip;
  cart_I.bottomRightCorner(3, 3) << Io;

  null_K.setIdentity();
  null_K = null_K_d;

  // ---------------------------------------------------------------------------
  // compute error
  // ---------------------------------------------------------------------------

  // Calculate vel = J*qdot (velocity of end-effector)
  Eigen::Matrix<double, 6, 1> vel(J * qdot);

  // Calculate position error
  error.head(3) << X0ee_d - X0ee;

  // Calculate orientation error
  Eigen::Matrix3d Rcd(R0ee_d * R0ee.transpose());
  error.tail(3) << R2r(Rcd);

  // Calculate velocity error
  vel_d.head(3) <<  X0ee_d - X0ee_d_prev; // desired velocity is derivative of desired position
  vel_error << vel_d - vel;
  X0ee_d_prev = X0ee_d;

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

  // Calculate error accumulated
  error_accum.head(3) = error_prev.head(3) + error.head(3); // position
  if (error.tail(3).transpose()*error.tail(3) < 0.00001) // orientation
    error_accum.tail(3) = error.tail(3);
  else
    error_accum.tail(3) = ( error_prev.tail(3).transpose() * (error.tail(3)/error.tail(3).norm()) * (error.tail(3)/error.tail(3).norm()) ) + error.tail(3);

  // Saturate integral error
  for (size_t i = 0; i < 3; i++) // position error
  {
    if (error_accum[i] > i_clamp_p)
      error_accum[i] = i_clamp_p;
    else if (error_accum[i] < -i_clamp_p)
      error_accum[i] = -i_clamp_p;
  }
  for (size_t i = 3; i < 6; i++) // orientation error
  {
    if (error_accum[i] > i_clamp_o)
      error_accum[i] = i_clamp_o;
    else if (error_accum[i] < -i_clamp_o)
      error_accum[i] = -i_clamp_o;
  }

  // Calculate task torque (tau_task)
  tau_task << J.transpose() * ( cart_D * vel_error + cart_K * error + cart_I * error_accum);

  error_prev = error_accum;

  // Calculate final torque
  tau_d << tau_task + tau_null + C + g;

  // Publish desired torques
  panda_3dbioprint_simulation::Tau tau_msg;
  geometry_msgs::Wrench wrench_msg;
  panda_3dbioprint_simulation::TrackingError error_msg;

  tau_msg.joint_name.resize(joint_handles.size());
  error_msg.error.resize(6);

  // Prepare Tau msg
  for (size_t i = 0; i < joint_handles.size(); i++)
    tau_msg.joint_name[i] = joint_handles[i].getName();

  tau_msg.tau_d.resize(tau_d.size());
  tau_msg.tau_task.resize(tau_task.size());
  tau_msg.tau_null.resize(tau_null.size());
  Eigen::VectorXd::Map(&tau_msg.tau_d[0], tau_d.size()) = tau_d;
  Eigen::VectorXd::Map(&tau_msg.tau_task[0], tau_task.size()) = tau_task;
  Eigen::VectorXd::Map(&tau_msg.tau_null[0], tau_null.size()) = tau_null;

  // Prepare force msg
  Eigen::VectorXd wrench = Eigen::VectorXd(6);
  wrench = Jhash.transpose() * (tau - tau_initial);
  wrench_msg.force.x = wrench[0]; 
  wrench_msg.force.y = wrench[1]; 
  wrench_msg.force.z = wrench[2];
  wrench_msg.torque.x = wrench[3]; 
  wrench_msg.torque.y = wrench[4]; 
  wrench_msg.torque.z = wrench[5]; 

  // Prepare tracking error msg
  Eigen::VectorXd::Map(&error_msg.error[0], error.size()) = error;

  // Publish the messages
  tauPub.publish(tau_msg);
  wrenchPub.publish(wrench_msg);
  errorPub.publish(error_msg);

  // Set desired torque to each joint
  if (!torques_2_robot) 
  {
    tau_d << 0, 0, 0, 0, 0, 0, 0;
  }

  for (size_t i = 0; i < 7; ++i)
  {
    joint_handles[i].setCommand(tau_d(i));
  }

  // filter position (removes accuracy)
  X0ee_d = filter_param * X0ee_d_target + (1.0 - filter_param) * X0ee_d;
  // filter orientation
  orient_d = orient_d.slerp(filter_param, orient_d_target);
  orient_d.normalize();
  R0ee_d = orient_d.toRotationMatrix();
}

bool CartesianImpedanceController::calcJacobian(KDL::Jacobian &Jac, const Eigen::Matrix<double, 7, 1> &q_in)
{
  if (q_in.size() != n_joints) return false;

  KDL::ChainJntToJacSolver solver(k_chain);
  KDL::JntArray joint_array(n_joints);
  
  for (size_t i = 0; i < n_joints; i++)
    joint_array(i) = q_in(i);

  // Obtain jacobian
  solver.JntToJac(joint_array, Jac);

  return true;
}

bool CartesianImpedanceController::jacobian(Eigen::Matrix<double, 6, 7> &J_out, const Eigen::Matrix<double, 7, 1> &q_in)
{
  KDL::Jacobian Jac(n_joints);

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

bool CartesianImpedanceController::fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf)
{
  if (q_in.size() != n_joints) return false;

  KDL::ChainFkSolverPos_recursive solver(k_chain);
  KDL::Frame end_effector_frame;
  KDL::JntArray joint_array(n_joints);

  for (size_t i = 0; i < n_joints; i++)
    joint_array(i) = q_in(i);

  // Obtain end-effector frame orientation and position within end_effector_frame variable
  solver.JntToCart(joint_array, end_effector_frame);

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      transf(i, j) = end_effector_frame(i, j);

  return true;
}

bool CartesianImpedanceController::dynamic(Eigen::Matrix<double, 7, 1>& q, Eigen::Matrix<double, 7, 1>& qdot)
{
  if(q.size() != n_joints || qdot.size() != n_joints){
    return false;
  }

  KDL::Vector g_vector = {0.0, 0.0, -9.80665};   // Gravity vector
  KDL::ChainDynParam chain_dyn(k_chain, g_vector);
  
  KDL::JntArray q_jarray(n_joints);
  KDL::JntArray qdot_jarray(n_joints);

  KDL::JntSpaceInertiaMatrix kdl_inertia;
  KDL::JntArray kdl_coriolis;
  KDL::JntArray kdl_gravity;

  // define inertia matrix with correct size (rows and columns)
  kdl_inertia = KDL::JntSpaceInertiaMatrix(n_joints);
  kdl_coriolis = KDL::JntArray(n_joints);
  kdl_gravity = KDL::JntArray(n_joints);

  for (size_t i = 0; i < n_joints; i++)
  {
    q_jarray(i) = q(i);
    qdot_jarray(i) = qdot(i);
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

  for (size_t i = 0; i < n_joints; i++){
    for (size_t j = 0; j < n_joints; j++){
      M(i,j) = kdl_inertia(i,j);
      C(i) = kdl_coriolis(i);
      g(i) = kdl_gravity(i);
    }
  }

  return true;
}

Eigen::Vector3d CartesianImpedanceController::R2r(Eigen::Matrix3d &Rotation)
{
  Eigen::Vector3d rotation_vector, aux;
  aux << Rotation(2, 1) - Rotation(1, 2), Rotation(0, 2) - Rotation(2, 0), Rotation(1, 0) - Rotation(0, 1);
  rotation_vector = 0.5 * aux;

  return rotation_vector;
}

void CartesianImpedanceController::DesiredGainsParamCallback(panda_3dbioprint_simulation::CartesianImpedanceControllerConfig &config, uint32_t level)
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
  i_clamp_p = config.Ip_clamp;
  i_clamp_o = config.Io_clamp;

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

  // position integral in desired frame
  Ip_d_target << Ipx,   0,   0,
                   0, Ipy,   0,
                   0,   0, Ipz;

  // orientation integral in desired frame
  Io_d_target << Iox,   0,   0,
                   0, Ioy,   0,
                   0,   0, Ioz;

  // nullspace Gains
  null_K_d_target = Kpn * null_K_d_target.setIdentity();
}

void CartesianImpedanceController::updatePoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
  X0ee_d_target << msg->position.x, msg->position.y, msg->position.z;

  Eigen::Quaterniond last_orient_d_target(orient_d_target);
  orient_d_target.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
  if (last_orient_d_target.coeffs().dot(orient_d_target.coeffs()) < 0.0)
    orient_d_target.coeffs() << -orient_d_target.coeffs();
}

bool CartesianImpedanceController::sendTorquesToRobot(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  torques_2_robot = req.data;
  res.success = true;
  return true;
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

}  // namespace panda_3dbioprint_simulation

PLUGINLIB_EXPORT_CLASS(panda_3dbioprint_simulation::CartesianImpedanceController, controller_interface::ControllerBase)