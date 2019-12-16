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
  if (!k_tree.getChain("panda_link0", "panda_hand", k_chain))
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
  // Set desired end-effector position and euler angles
  Eigen::Matrix<double, 7, 1> qd;
  qd << 0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4;
  
  if (!fk(qd, T0ee_d))
    ROS_ERROR("Cannot get forward kinematics.");

  Eigen::Affine3d transform(T0ee_d);
  X_d = transform.translation();
  R_d = transform.rotation();

  X_d_target = X_d;
  orient_d_target = R_d;
}

void CartesianImpedanceController::update(const ros::Time &time, const ros::Duration &period)
{
  // Obtain joint positions (q), velocities (qdot) and efforts (effort) from hardware interface
  for (size_t i = 0; i < n_joints; i++)
  {
    q(i) = joint_handles[i].getPosition();
    qdot(i) = joint_handles[i].getVelocity();
  }

  // Calculate X = f(q) using forward kinematics (FK)
  if (!fk(q, T0ee))
    ROS_ERROR("Cannot get forward kinematics.");
    
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
  // Eigen::Matrix3d Rcd(R_d * R.transpose()); // old (also works)
  Eigen::Matrix3d cRcd(R.transpose() * R_d);
  Eigen::Matrix3d Rcd(R * cRcd * R.transpose());
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
  tau_d << tau_task + tau_null + C + g;

  // Set desired torque to each joint
  for (size_t i = 0; i < 7; ++i)
  {
    joint_handles[i].setCommand(tau_d(i));
  }
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

}  // namespace panda_3dbioprint_simulation

PLUGINLIB_EXPORT_CLASS(panda_3dbioprint_simulation::CartesianImpedanceController, controller_interface::ControllerBase)