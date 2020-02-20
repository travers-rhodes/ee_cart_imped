#ifndef __EE_CART_IMPED_CONTROL_HPP__
#define __EE_CART_IMPED_CONTROL_HPP__

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/SVD>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <ros/ros.h>
#include <ee_cart_imped_msgs/EECartImpedAction.h>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>
#include <ee_cart_imped_msgs/StiffPoint.h>

namespace ee_cart_imped_control_ns {

  /**
   * \brief Allows a user to control the force or stiffness applied by joints.
   *
   * This class contains the realtime controller for controlling joints
   * by stiffness or force.  To use this controller, you should use the
   * action wrapper, 
   *<A href=http://www.ros.org/wiki/ee_cart_imped_action>EECartImpedAction</A>.
   */
  class EECartImpedControlClass: public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  private:
    
    /// The interface to the robot hardware
    hardware_interface::EffortJointInterface* hardware_interface_;
    std::vector<hardware_interface::JointHandle> joints_;
    // (For gazebo especially) we need ot make sure our joint effort commands are valid
    std::vector<joint_limits_interface::JointLimits> limits_list_;
    // the max effort per joint
    std::vector<double> max_effort_;
    
    /// The chain of links and joints in KDL language
    // Read-only after initialization
    KDL::Chain kdl_chain_;
    
    /// KDL Solver performing the joint angles to Cartesian pose calculation
    // Referenced only in update loop
    boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    /// KDL Solver performing the joint angles to Jacobian calculation
    // Referenced only in update loop
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    boost::scoped_ptr<KDL::ChainDynParam> kdl_chain_dyn_param_;

    /// KDL gravity compensation helpers
    // gravity, relative to KDL chain base link
    KDL::Vector grav_vector_; 
    // Referenced only in update loop
    KDL::JntArray  jnt_gravity_; 
    
    // The variables (which are declared as class variables
    // because they need to be pre-allocated, meaning we must
    // be very careful how we access them to avoid race conditions)
    
    /// Joint positions
    // Referenced only in update loop
    KDL::JntArray  q_; 
    /// Joint velocities
    // Referenced only in update loop
    KDL::JntArrayVel  qdot_;
    /// Joint torques   
    // Referenced only in update loop
    KDL::JntArray  tau_, tau_act_;
    
    /// Tip pose
    // Referenced only in update loop
    KDL::Frame     x_; 
    /// Tip desired pose          
    // Referenced only in update loop
    KDL::Frame     xd_;
    
    /// Cartesian error
    // Referenced only in update loop
    KDL::Twist     xerr_;
    /// Cartesian velocity
    // Referenced only in update loop
    KDL::Twist     xdot_;
    /// Cartesian effort
    // Referenced only in update loop
    KDL::Wrench    F_;  
    /// Desired Cartesian force
    // Referenced only in update loop
    KDL::Wrench    Fdes_;
    /// Jacobian
    // Referenced only in update loop
    KDL::Jacobian  J_;         

    Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner> svd_;

    // whether the X,Y,Z (cartesian) and X,Y,Z (euler angles) values are specifying torque
    // or whether we are in position control on that joint instead
    std::array<bool,6> isForceTorqueArray_;

    // The default stiffness (used when controller freezes)
    double default_stiffness_;
    // The default rotational stiffness (used when controller freezes)
    double default_rotational_stiffness_;
    // The coefficient of the joint-level dampening term
    // (Maybe we can set this to zero because there's a lot of friction on joints)
    double joint_dampening_;

    ///The time at which the goal we saw on the last iteration was started
    //Referenced only in update loop
    double last_goal_starting_time_;
    
    typedef std::vector<ee_cart_imped_msgs::StiffPoint> EECartImpedTraj;
    
    ///Class for storing a trajectory and associated information
    //This is a class so as to be able to lock and unlock this data
    //together easily
    class EECartImpedData {
    public:
      ///Desired trajectory
      EECartImpedTraj traj;
      ///The point the robot was at before beginning trajectory
      ee_cart_imped_msgs::StiffPoint initial_point;
      ///The starting time of the trajectory
      ros::Time starting_time;
      EECartImpedData() {}
    };

    ///Desired positions
    //This needs to be in realtime boxes because
    //we reference it in the update loop and modify 
    //it it in the command callback
    realtime_tools::RealtimeBox<
      boost::shared_ptr<const EECartImpedData> > desired_poses_box_;
    
    ///The actual position of the robot when we achieved the last point on the trajectory
    //Referenced only in updated loop
    ee_cart_imped_msgs::StiffPoint last_point_;

    // Note the gains are incorrectly typed as a twist,
    // as there is no appropriate type!
    /// Proportional gains
    //Referenced only in update loop
    KDL::Twist     Kp_;
    /// Derivative gains
    //Referenced only in update loop
    KDL::Twist     Kd_;   
    
    /// Time of the last servo cycle
    //Modified in both command callback
    //and update loop, but it does not matter
    ros::Time last_time_;

    /**
     * \brief Linearly interpolates between trajectory points
     *
     * Linearly interpolates between startValue and endValue over 
     * the time period startTime to endTime of the current goal.
     * @param time [seconds] that this goal has been running
     * @param startTime time that the goal began running
     * @param endTime time that the goal should finish running
     * @param startValue the values at the beginning of the interpolation (i.e. the values at startTime)
     * @param endValue the values at the end of the interpolation (i.e. the values at endTime)
     * @return ((time - startTime) * (endValue - startValue) / 
     (endTime - startTime)) + startValue; 
    */
    
    double linearlyInterpolate(double time, double startTime, 
			       double endTime, double startValue, 
			       double endValue);
    
    
    /**
     *\brief Callback when a new goal is received
     *
     *Cancels the currently active goal (if there is one), restarts
     *the controller and asks it to execute the new goal.
     *
     *@param msg the new goal
     *
     */
    void commandCB(const ee_cart_imped_msgs::EECartImpedGoalConstPtr &msg);
    ros::NodeHandle node_;
    ros::Subscriber subscriber_;
    

    /**
     *\brief Samples the interpolation and returns the point the
     * joints should currently be trying to achieve.
     *
     * Samples the interpolation, using 
     * EECartImpedControlClass::linearlyInterpolate, between the
     * current trajectory point and the next trajectory point.  Interpolation
     * is done for position and orientation, but not for wrench or stiffness
     * as generally it is desired that wrench or stiffness be constant
     * over a specific trajectory segment.
     *
     * @return A <A HREF=http://www.ros.org/doc/api/ee_cart_imped_msgs/html/msg/StiffPoint.html>StiffPoint</A> that is the point
     * the joints should attempt to achieve on this timestep.
     *
     */
    ee_cart_imped_msgs::StiffPoint sampleInterpolation(const ros::Time& time);
    
    ///State publisher, published every 10 updates
    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
	ee_cart_imped_msgs::EECartImpedFeedback> > 
    controller_state_publisher_;
    
    ///The number of updates to the joint position
    //Referenced only in updates
    int updates_;

    /**
     * Sets the desired
     * position and orientation to be the current position and orientation
     * and sets the joints to have maximum stiffness so that they hold
     * their current position.
     */
    void hold_current_pose(const ros::Time& time);

    bool constructKDLChain(std::string root, std::string tip, std::string robot_desc_string);

  public:
    /**
     * \brief Controller initialization in non-realtime
     *
     * Initializes the controller on first startup.  Prepares the 
     * kinematics, pre-allocates the variables, subscribes to
     * command and advertises state.
     *
     * @param robot The current robot state
     * @param n A node handle for subscribing and advertising topics
     *
     * @return True on successful initialization, false otherwise
     *
     */
    bool init(hardware_interface::EffortJointInterface *robot,
	      ros::NodeHandle &n);
    /**
     * \brief Controller startup in realtime
     * 
     * Resets the controller to prepare for a new goal.  Sets the desired
     * position and orientation to be the current position and orientation
     * and sets the joints to have maximum stiffness so that they hold
     * their current position.
     */
    void starting(const ros::Time& time);
    
    /**
     * \brief Controller update loop in realtime
     *
     * A PD controller for achieving the trajectory.
     * Uses EECartImpedControlClass::sampleInterpolation to
     * find the point that should be achieved on the current timestep.
     * Converts this point (which is in Cartesian coordinates) to joint
     * angles and uses a PD update (in the case of stiffness) to update 
     * the joint array values to the correct force.
     *
     */
    void update(const ros::Time& time, const ros::Duration& period) override;

    /**
     * \brief Controller stopping in realtime
     *
     * Calls EECartImpedControlClass::hold_current_pose() to lock the joints into
     * their current position.
     *
     */
    void stopping(const ros::Time& time);
  };
}

#endif //ee_cart_imped_control.hpp
