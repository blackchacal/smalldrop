#include <ros/ros.h>
#include <ros/node_handle.h>

#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <panda_3dbioprint_real/CartesianImpedanceControllerConfig.h>

#include <Eigen/Dense>
#include <fstream>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>

#include "franka_hw/franka_model_interface.h"
#include "franka_hw/franka_state_interface.h"

namespace panda_3dbioprint_real
{
  class CartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                    hardware_interface::EffortJointInterface,
                                                    franka_hw::FrankaStateInterface,
                                                    franka_hw::FrankaModelInterface>
  {
  private:
    float filter_param = 0.01;
    const double delta_tau_max = 1.0;
    
    // Robot general variables
    unsigned int n_joints;
    std::vector<hardware_interface::JointHandle> joint_handles;
    std::map<std::string, urdf::JointSharedPtr> robot_joints;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle;

    // Robot Kinematics variables
    Eigen::Matrix4d T0ee, T0ee_d;
    Eigen::Matrix<double, 7, 1> q, qdot, tau;
    Eigen::Vector3d X_d, X_d_target;
    Eigen::Matrix3d R_d, R_d_target;
    Eigen::Quaterniond orient_d, orient_d_target;
    Eigen::Matrix<double, 6, 1> vel_d, error, vel_error;

    // Robot Dynamics variables
    Eigen::Matrix<double, 6, 7> J;   // Jacobian
    Eigen::Matrix<double, 7, 6> Jhash;  // Dynamically consistent generalized inverse of the jacobian
    Eigen::Matrix<double, 7, 7> M;  // inertia or mass
    Eigen::Matrix<double, 7, 1> C;  // coriolis
    Eigen::Matrix<double, 7, 1> g;  // gravity
    Eigen::Matrix<double, 6, 6> cart_K; // cartesian stiffness
    Eigen::Matrix<double, 6, 6> cart_D; // cartesian damping
    Eigen::Matrix<double, 7, 7> null_K; // nullspace stiffness
    Eigen::Matrix3d Kp_d, Dp_d, Kp_d_target, Dp_d_target; // position stiffness and damping in desired frame
    Eigen::Matrix3d Ko_d, Do_d, Ko_d_target, Do_d_target; // orientation stiffness and damping in desired frame
    double Kpx, Kpy, Kpz, Kox, Koy, Koz, Dpx, Dpy, Dpz, Dox, Doy, Doz, Kpn;

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<panda_3dbioprint_real::CartesianImpedanceControllerConfig>> dyn_config_gains_param;
    ros::NodeHandle dyn_config_gains_node;

    // Publisher/Subscriber
    ros::Publisher posePub;
    ros::Subscriber poseSub;

    // File handlers
    std::ifstream fh_gains;

    // Method
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);
    Eigen::Vector3d R2r(const Eigen::Matrix3d &Rotation);
    
    // Callback Methods
    void DesiredGainsParamCallback(panda_3dbioprint_real::CartesianImpedanceControllerConfig &config, uint32_t level);
    void updatePoseCallback(const geometry_msgs::PoseConstPtr &msg);

    // posture optimization ----------------------------------------------------
    Eigen::Matrix<double, 7, 1> maxJointLimits;
    Eigen::Matrix<double, 7, 1> minJointLimits;
    Eigen::Matrix<double, 7, 1> gradient;  // gradient mechanical joint limit
    double derivative_computation( const double q_i, const double maxJointLimit_i, const double minJointLimit_i);
    template<int N> // number of joints or DOF
    void gradient_mechanical_joint_limit( Eigen::Matrix<double, N, 1>& gradient_mechanical_joint_limit_out, const Eigen::Matrix<double, N, 1> q, const Eigen::Matrix<double, N, 1> maxJointLimits, const Eigen::Matrix<double, N, 1> minJointLimits );
    
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;
    void starting(const ros::Time &time) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
  };
   
} // namespace panda_3dbioprint_real