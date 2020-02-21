// For class and function comments, see 
// include/ee_cart_imped_control/ee_cart_imped_control.hpp
// or the API docs
// 
// The structure of this code borrows from the Realtime controller
// KDL tutorial and the joint trajectory spline controller

// A modern example of this type of file is available at
// https://github.com/ros-controls/ros_controllers/blob/melodic-devel/effort_controllers/src/joint_position_controller.cpp

#include "ee_cart_imped_control/ee_cart_imped_control.hpp"
#include <pluginlib/class_list_macros.h>

using namespace ee_cart_imped_control_ns; 
double EECartImpedControlClass::linearlyInterpolate(double time, 
    double startTime, 
    double endTime, 
    double startValue, 
    double endValue) {
  return startValue + 
    (time - startTime)*
    (endValue - startValue)/(endTime - startTime);
}

ee_cart_imped_msgs::StiffPoint 
EECartImpedControlClass::sampleInterpolation(const ros::Time& current_time) {
  boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
  desired_poses_box_.get(desired_poses_ptr);
  if (!desired_poses_ptr) {
    ROS_FATAL("ee_cart_imped_control: current trajectory was NULL!");
  }
  const EECartImpedTraj &desiredPoses = desired_poses_ptr->traj;
  const ee_cart_imped_msgs::StiffPoint &initial_point = 
    desired_poses_ptr->initial_point;
  const ros::Time &current_goal_start_time = desired_poses_ptr->starting_time;
  if (desiredPoses.size() == 0) {
    ROS_ERROR("ee_cart_imped_control: Empty trajectory");
    return last_point_;
  }
  if (last_goal_starting_time_ != current_goal_start_time.toSec()) {
    //we have never seen this goal before
    last_point_ = initial_point;
    last_point_.time_from_start = ros::Duration(0);
  }
  last_goal_starting_time_ = current_goal_start_time.toSec();

  // time from the start of the series of points
  double time = current_time.toSec();
  double timeFromStart = time - current_goal_start_time.toSec();  
  ee_cart_imped_msgs::StiffPoint next_point;


  //Find the correct trajectory segment to use
  //We don't want a current_goal_index_ because it has
  //the potential to get caught in a bad race condition
  int current_goal_index;
  for (current_goal_index = 0; 
      current_goal_index < (signed int)desiredPoses.size();
      current_goal_index++) {
    if (desiredPoses[current_goal_index].time_from_start.toSec() >= timeFromStart) {
      break;
    }
  }

  if (current_goal_index >= (signed int)desiredPoses.size()) {
    //we're done with the goal, hold the last position
    return desiredPoses[current_goal_index-1];
  }

  //did we move on to the next point?
  if (current_goal_index > 0 && last_point_.time_from_start.toSec() != 
      desiredPoses[current_goal_index-1].time_from_start.toSec()) {
    //this should be where we CURRENTLY ARE
    last_point_.pose.position.x = x_.p(0);
    last_point_.pose.position.y = x_.p(1);
    last_point_.pose.position.z = x_.p(2);
    x_.M.GetQuaternion(last_point_.pose.orientation.x,
        last_point_.pose.orientation.y,
        last_point_.pose.orientation.z,
        last_point_.pose.orientation.w);
    last_point_.wrench_or_stiffness = 
      desiredPoses[current_goal_index-1].wrench_or_stiffness;
    last_point_.isForceX = desiredPoses[current_goal_index-1].isForceX;
    last_point_.isForceY = desiredPoses[current_goal_index-1].isForceY;
    last_point_.isForceZ = desiredPoses[current_goal_index-1].isForceZ;
    last_point_.isTorqueX = desiredPoses[current_goal_index-1].isTorqueX;
    last_point_.isTorqueY = desiredPoses[current_goal_index-1].isTorqueY;
    last_point_.isTorqueZ = desiredPoses[current_goal_index-1].isTorqueZ;
    last_point_.time_from_start = 
      desiredPoses[current_goal_index-1].time_from_start;
  }

  ee_cart_imped_msgs::StiffPoint start_point;
  const ee_cart_imped_msgs::StiffPoint &end_point = 
    desiredPoses[current_goal_index];
  //actually now last_point_ and initial point should be the
  //same if current_goal_index is zero
  if (current_goal_index == 0) {
    start_point = initial_point;
  } else { 
    start_point = last_point_;
  }

  double segStartTime = 0.0;
  if (current_goal_index > 0) {
    segStartTime = 
      desiredPoses[current_goal_index-1].time_from_start.toSec();
  }
  double segEndTime = 
    desiredPoses[current_goal_index].time_from_start.toSec();
  if (segEndTime <= segStartTime) {
    //just stay where we currently are
    next_point.pose.position.x = x_.p(0);
    next_point.pose.position.y = x_.p(1);
    next_point.pose.position.z = x_.p(2);
    x_.M.GetQuaternion(next_point.pose.orientation.x,
        next_point.pose.orientation.y,
        next_point.pose.orientation.z,
        next_point.pose.orientation.w);

  } else {
    next_point.pose.position.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.x, end_point.pose.position.x);

    next_point.pose.position.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.y, end_point.pose.position.y); 

    next_point.pose.position.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.z, end_point.pose.position.z); 

    next_point.pose.orientation.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.x, end_point.pose.orientation.x); 

    next_point.pose.orientation.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.y, end_point.pose.orientation.y);

    next_point.pose.orientation.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.z, end_point.pose.orientation.z); 

    next_point.pose.orientation.w = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.w, end_point.pose.orientation.w); 
  }
  //we don't currently interpolate between wrench
  //and stiffness as generally the user wants one
  //wrench through a full trajectory point and then
  //another at the next trajectory point
  //perhaps, however, we should put in some interpolation
  //to avoid fast transitions between very different wrenches
  //as these can lead to bad behavior
  next_point.wrench_or_stiffness = 
    desiredPoses[current_goal_index].wrench_or_stiffness;
  next_point.isForceX = desiredPoses[current_goal_index].isForceX;
  next_point.isForceY = desiredPoses[current_goal_index].isForceY;
  next_point.isForceZ = desiredPoses[current_goal_index].isForceZ;
  next_point.isTorqueX = desiredPoses[current_goal_index].isTorqueX;
  next_point.isTorqueY = desiredPoses[current_goal_index].isTorqueY;
  next_point.isTorqueZ = desiredPoses[current_goal_index].isTorqueZ;
  next_point.time_from_start = ros::Duration(timeFromStart);
  return next_point;
}

void EECartImpedControlClass::commandCB
(const ee_cart_imped_msgs::EECartImpedGoalConstPtr &msg) {
  if ((msg->trajectory).empty()) {
    //stop the controller
    ros::Time time_now = ros::Time::now();
    hold_current_pose(time_now);
    return;
  }
  //this is a new goal
  boost::shared_ptr<EECartImpedData> new_traj_ptr
    (new EECartImpedData());
  if (!new_traj_ptr) {
    ROS_ERROR("Null new trajectory.");
    ros::Time time_now = ros::Time::now();
    hold_current_pose(time_now);
    return;
  }

  EECartImpedData &new_traj = *new_traj_ptr;
  KDL::Frame init_pos;
  KDL::JntArray q0(kdl_chain_.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);

  for (int i = 0; i < joints_.size(); i++) {
    q0(i) = joints_[i].getPosition();    
  }

  fksolver.JntToCart(q0, init_pos);

  new_traj.initial_point.pose.position.x = init_pos.p(0);
  new_traj.initial_point.pose.position.y = init_pos.p(1);
  new_traj.initial_point.pose.position.z = init_pos.p(2);
  init_pos.M.GetQuaternion(new_traj.initial_point.pose.orientation.x,
      new_traj.initial_point.pose.orientation.y,
      new_traj.initial_point.pose.orientation.z,
      new_traj.initial_point.pose.orientation.w);

  for (size_t i = 0; i < msg->trajectory.size(); i++) {
    new_traj.traj.push_back(msg->trajectory[i]);
  }
  if (!new_traj_ptr) {
    ROS_ERROR("Null new trajectory after filling.");
    ros::Time time_now = ros::Time::now();
    hold_current_pose(time_now);
    return;
  }
  new_traj.starting_time = ros::Time::now();
  desired_poses_box_.set(new_traj_ptr);
}

bool EECartImpedControlClass::init(hardware_interface::EffortJointInterface *robot,
    ros::NodeHandle &n) {
  ROS_INFO("Started initializing control class");
  std::string root_name, tip_name;
  node_ = n;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: %s)",
        n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
        n.getNamespace().c_str());
    return false;
  }
  std::vector<double> cartesian_dampening;
  if (!n.getParam("cartesian_dampening", cartesian_dampening))
  {
    double default_cartesian_dampening = 10;
    ROS_WARN("No cartesian_dampening given in namespace: %s. Defaulting to %f.",
        n.getNamespace().c_str(), default_cartesian_dampening);
    cartesian_dampening.clear();
    for (int i = 0; i< 3; i++) {
      cartesian_dampening.push_back(default_cartesian_dampening);
    }
  }
  if (cartesian_dampening.size() != 3) {
    ROS_ERROR("cartesian_dampening must be a vector of length 3");
    return false;
  }
  std::vector<double> euler_angle_dampening;
  if (!n.getParam("euler_angle_dampening", euler_angle_dampening))
  {
    double default_euler_angle_dampening = 1;
    ROS_WARN("No euler_angle_dampening given in namespace: %s. Defaulting to %f.",
        n.getNamespace().c_str(), default_euler_angle_dampening);
    euler_angle_dampening.clear();
    for (int i = 0; i< 3; i++) {
      euler_angle_dampening.push_back(default_euler_angle_dampening);
    }
  }
  if (euler_angle_dampening.size() != 3) {
    ROS_ERROR("euler_angle_dampening must be a vector of length 3");
    return false;
  }
  if (!n.getParam("default_stiffness", default_stiffness_))
  {
    double default_default_stiffness = 1000;
    ROS_WARN("No default_stiffness given in namespace: %s. Defaulting to %f.",
        n.getNamespace().c_str(), default_default_stiffness);
    default_stiffness_ = default_default_stiffness;
  }
  if (!n.getParam("default_rotational_stiffness", default_rotational_stiffness_))
  {
    double default_default_rotational_stiffness = 1000;
    ROS_WARN("No default_rotational_stiffness given in namespace: %s. Defaulting to %f.",
        n.getNamespace().c_str(), default_default_rotational_stiffness);
    default_rotational_stiffness_ = default_default_rotational_stiffness;
  }
  if (!n.getParam("joint_dampening", joint_dampening_))
  {
    double default_joint_dampening = 0.5;
    ROS_WARN("No joint_dampening given in namespace: %s. Defaulting to %f.",
        n.getNamespace().c_str(), default_joint_dampening);
    joint_dampening_ = default_joint_dampening;
  }
  if (!n.getParam("use_jacobian_inverse", use_jacobian_inverse_))
  {
    bool default_use_jacobian_inverse= false;
    ROS_WARN("No use_jacobian_inverse given in namespace: %s. Defaulting to %d.",
        n.getNamespace().c_str(), default_use_jacobian_inverse);
    use_jacobian_inverse_ = default_use_jacobian_inverse;
  }
  if (!n.getParam("least_squares_dampening", least_squares_dampening_) && use_jacobian_inverse_)
  {
    double default_least_squares_dampening = 0.0001;
    ROS_WARN("No least_squares_dampening given in namespace: %s. Defaulting to %f.",
        n.getNamespace().c_str(), default_least_squares_dampening);
    least_squares_dampening_ = default_least_squares_dampening;
  }

  // Store the hardware_interface handle for later use (to get time)
  hardware_interface_ = robot;
  
  
  urdf::Model urdf_robot;
  // load urdf from "robot_description"
  // TODO don't hard-code this string
  urdf_robot.initParam("/robot_description");

  ROS_INFO("Constructing KDL chain");
  // Constructs kdl_chain_ parameter and joints_ parameter
  std::string robot_desc_string;
  n.getParam("/robot_description", robot_desc_string);
  if (!constructKDLChain(root_name, tip_name, robot_desc_string)) {
    ROS_ERROR("Couldn't construct chain from %s to %s.)",
        root_name.c_str(), tip_name.c_str());
    return false;
  }
 
  // TODO: Pull this from KDLChain somehow
  std::vector<std::string> joint_names = {
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7"
  };

  for (int i = 0; i < joint_names.size(); i++)
  {
    joint_limits_interface::JointLimits limits;
    std::shared_ptr<const urdf::Joint> urdf_joint = urdf_robot.getJoint(joint_names[i]);
    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(urdf_joint, limits);
    // Populate joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' 
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = joint_limits_interface::getJointLimits(joint_names[i], n, limits);
    limits_list_.push_back(limits);
  }
  
  for (int i = 0; i < joint_names.size(); i++)
  {
    if (limits_list_[i].has_effort_limits) {
      max_effort_.push_back(limits_list_[i].max_effort);
    } else {
      double default_effort_limit;
      ROS_WARN("No effort limit found for joint %s, using default of %f.", joint_names[i].c_str(), default_effort_limit);
      max_effort_.push_back(default_effort_limit);
    }
  }

  ROS_INFO("Prepare svd calculator");
  // VERY conservative here.
  svd_.setThreshold(0.001);


  ROS_INFO("Convert chain to KDL chain");
  // Construct the kdl solvers in non-realtime
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  grav_vector_ = KDL::Vector(0., 0., -9.81);
  kdl_chain_dyn_param_.reset(new KDL::ChainDynParam(kdl_chain_, grav_vector_));

  // Resize (pre-allocate) the variables in non-realtime
  q_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  tau_act_.resize(kdl_chain_.getNrOfJoints());
  J_.resize(kdl_chain_.getNrOfJoints());
  jnt_gravity_.resize(kdl_chain_.getNrOfJoints());

  subscriber_ = node_.subscribe("command", 1, &EECartImpedControlClass::commandCB, this);
  controller_state_publisher_.reset
    (new realtime_tools::RealtimePublisher
     <ee_cart_imped_msgs::EECartImpedFeedback>
     (node_, "state", 1));
  controller_state_publisher_->msg_.requested_joint_efforts.resize
    (kdl_chain_.getNrOfJoints());
  controller_state_publisher_->msg_.actual_joint_efforts.resize
    (kdl_chain_.getNrOfJoints());
  updates_ = 0;


  Kd_.vel(0) = cartesian_dampening[0];        // Translation x                                                   
  Kd_.vel(1) = cartesian_dampening[1];        // Translation y
  Kd_.vel(2) = cartesian_dampening[2];        // Translation z
  Kd_.rot(0) = euler_angle_dampening[0];        // Rotation x
  Kd_.rot(1) = euler_angle_dampening[1];        // Rotation y
  Kd_.rot(2) = euler_angle_dampening[2];        // Rotation z

  //Create a dummy trajectory
  boost::shared_ptr<EECartImpedData> dummy_ptr(new EECartImpedData());
  EECartImpedTraj &dummy = dummy_ptr->traj;
  dummy.resize(1);
  dummy[0].time_from_start = ros::Duration(0);
  desired_poses_box_.set(dummy_ptr);
  last_goal_starting_time_ = -1;
  return true;
}

//
// This function creates the first command to the robot which is the robot's current position
// And asks the robot to hold this position
//  
void EECartImpedControlClass::hold_current_pose(const ros::Time& current_time) {
  // Get the current joint values to compute the initial tip location.
  KDL::Frame init_pos;
  KDL::JntArray q0(kdl_chain_.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
  for (int i = 0; i < joints_.size(); i++) {
    q0(i) = joints_[i].getPosition();    
  }
  fksolver.JntToCart(q0, init_pos);


  // Also reset the time-of-last-servo-cycle
  last_time_ = current_time; 
  ///Hold current position trajectory
  boost::shared_ptr<EECartImpedData> hold_traj_ptr(new EECartImpedData());
  if (!hold_traj_ptr) {
    ROS_ERROR("While starting, trajectory pointer was null");
    return;
  }
  EECartImpedData &hold_traj = *hold_traj_ptr;
  hold_traj.traj.resize(1);

  hold_traj.traj[0].pose.position.x = init_pos.p(0);
  hold_traj.traj[0].pose.position.y = init_pos.p(1);
  hold_traj.traj[0].pose.position.z = init_pos.p(2);
  init_pos.M.GetQuaternion((hold_traj.traj[0].pose.orientation.x), 
      (hold_traj.traj[0].pose.orientation.y), 
      (hold_traj.traj[0].pose.orientation.z), 
      (hold_traj.traj[0].pose.orientation.w));
  hold_traj.traj[0].wrench_or_stiffness.force.x = default_stiffness_;
  hold_traj.traj[0].wrench_or_stiffness.force.y = default_stiffness_;
  hold_traj.traj[0].wrench_or_stiffness.force.z = default_stiffness_;
  hold_traj.traj[0].wrench_or_stiffness.torque.x = default_rotational_stiffness_;
  hold_traj.traj[0].wrench_or_stiffness.torque.y = default_rotational_stiffness_;
  hold_traj.traj[0].wrench_or_stiffness.torque.z = default_rotational_stiffness_;
  hold_traj.traj[0].isForceX = false;
  hold_traj.traj[0].isForceY = false;
  hold_traj.traj[0].isForceZ = false;
  hold_traj.traj[0].isTorqueX = false;
  hold_traj.traj[0].isTorqueY = false;
  hold_traj.traj[0].isTorqueZ = false;
  hold_traj.traj[0].time_from_start = ros::Duration(0);
  hold_traj.initial_point = hold_traj.traj[0];
  hold_traj.starting_time = ros::Time::now();
  if (!hold_traj_ptr) {
    ROS_ERROR("During starting hold trajectory was null after filling");
    return;
  }
  //Pass the trajectory through to the update loop
  desired_poses_box_.set(hold_traj_ptr);
}

void EECartImpedControlClass::update(const ros::Time& time, const ros::Duration& period)
{
  double prepare_time, their_ik_time, ik_time, gravity_time, after_time;
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = ros::Time::now();
  ros::Time very_start_time = start_time;
  ROS_WARN_THROTTLE(1.0, "UpdateLoop: Update period is %f", (time-last_time_).toSec());
  last_time_ = time;

  // Get the current joint positions and velocities
  // q_ and qdot_ are already initialized to the correct sizes
  for (int i = 0; i < joints_.size(); i++)
  {
    q_(i) = joints_[i].getPosition();
    qdot_.q(i) = joints_[i].getPosition();
    qdot_.qdot(i) = joints_[i].getVelocity();
  }

  // Compute the forward kinematics and Jacobian (at this location)
  jnt_to_pose_solver_->JntToCart(q_, x_);
  jnt_to_jac_solver_->JntToJac(q_, J_);

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
      xdot_(i) += J_(i,j) * qdot_.qdot(j);
  }

  ee_cart_imped_msgs::StiffPoint desiredPose = sampleInterpolation(time);


  Fdes_(0) = desiredPose.wrench_or_stiffness.force.x;
  Fdes_(1) = desiredPose.wrench_or_stiffness.force.y;
  Fdes_(2) = desiredPose.wrench_or_stiffness.force.z;
  Fdes_(3) = desiredPose.wrench_or_stiffness.torque.x;
  Fdes_(4) = desiredPose.wrench_or_stiffness.torque.y;
  Fdes_(5) = desiredPose.wrench_or_stiffness.torque.z;

  Kp_.vel(0) = desiredPose.wrench_or_stiffness.force.x;
  Kp_.vel(1) = desiredPose.wrench_or_stiffness.force.y;
  Kp_.vel(2) = desiredPose.wrench_or_stiffness.force.z;
  Kp_.rot(0) = desiredPose.wrench_or_stiffness.torque.x;
  Kp_.rot(1) = desiredPose.wrench_or_stiffness.torque.y;
  Kp_.rot(2) = desiredPose.wrench_or_stiffness.torque.z;

  xd_.p(0) = desiredPose.pose.position.x;
  xd_.p(1) = desiredPose.pose.position.y;
  xd_.p(2) = desiredPose.pose.position.z;
  xd_.M = KDL::Rotation::Quaternion(desiredPose.pose.orientation.x, 
      desiredPose.pose.orientation.y, 
      desiredPose.pose.orientation.z, 
      desiredPose.pose.orientation.w);

  // Calculate a Cartesian restoring force.
  xerr_.vel = x_.p - xd_.p;
  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
      xd_.M.UnitY() * x_.M.UnitY() +
      xd_.M.UnitZ() * x_.M.UnitZ());  // I have no idea what this is


  // F_ is a vector of forces/wrenches corresponding to x, y, z, tx,ty,tz,tw
  if (desiredPose.isForceX) {
    isForceTorqueArray_[0] = true;
    F_(0) = Fdes_(0);
  } else {
    isForceTorqueArray_[0] = false;
    F_(0) = -Kp_(0) * xerr_(0) - Kd_(0)*xdot_(0);
  }

  if (desiredPose.isForceY) {
    isForceTorqueArray_[1] = true;
    F_(1) = Fdes_(1);
  } else {
    isForceTorqueArray_[1] = false;
    F_(1) = -Kp_(1) * xerr_(1) - Kd_(1)*xdot_(1);
  }

  if (desiredPose.isForceZ) {
    isForceTorqueArray_[2] = true;
    F_(2) = Fdes_(2);
  } else {
    isForceTorqueArray_[2] = false;
    F_(2) = -Kp_(2) * xerr_(2) - Kd_(2)*xdot_(2);
  }

  if (desiredPose.isTorqueX) {
    isForceTorqueArray_[3] = true;
    F_(3) = Fdes_(3);
  } else {
    isForceTorqueArray_[3] = false;
    F_(3) = -Kp_(3) * xerr_(3) - Kd_(3)*xdot_(3);
  }

  if (desiredPose.isTorqueY) {
    isForceTorqueArray_[4] = true;
    F_(4) = Fdes_(4);
  } else {
    isForceTorqueArray_[4] = false;
    F_(4) = -Kp_(4) * xerr_(4) - Kd_(4)*xdot_(4);
  }

  if (desiredPose.isTorqueZ) {
    isForceTorqueArray_[5] = true;
    F_(5) = Fdes_(5);
  } else {
    isForceTorqueArray_[5] = false;
    F_(5) = -Kp_(5) * xerr_(5) - Kd_(5)*xdot_(5);
  }

  end_time = ros::Time::now();
  prepare_time = (end_time - start_time).toSec();
  start_time = end_time;

  // Solve for position control for non-force-controlled joints
  if (use_jacobian_inverse_)
  {
    svd_.compute(J_.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
  }

  end_time = ros::Time::now();
  their_ik_time = (end_time - start_time).toSec();
  start_time = end_time;

  // Convert the force into a set of joint torques
  // tau_ is a vector of joint torques q1...qn
  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) {
    // iterate through the vector.  every joint torque is contributed to
    // by the Jacobian Transpose (note the index switching in J access) times
    // the desired force (from impedance OR explicit force)

    // So if a desired end effector wrench is specified, there is no position control on that dof
    // if a wrench is not specified, then there is impedance based (basically p-gain) 
    //position control on that dof
    tau_(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++) {
      if (!use_jacobian_inverse_ || isForceTorqueArray_[j])
      {
        tau_(i) += J_(j,i) * F_(j);
      }
      else
      {
        // svd_.singularValues.length is 6, since we have more joints than dof
        for (unsigned int k = 0; k < 6; k++)
        {
           if (svd_.singularValues()[k] > 0.0001){
             // https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
             // damped least squares
             tau_(i) += svd_.matrixV()(i,k) * svd_.singularValues()[k]/(svd_.singularValues()[k]*svd_.singularValues()[k]+least_squares_dampening_) * svd_.matrixU()(j,k) * F_(j);
           }  
        }
      }
    }
  }

  // Joint-level dampening
  for (int i = 0; i < joints_.size(); i++)
  {
    tau_(i) -= qdot_.qdot(i) * joint_dampening_;
  }
  
  end_time = ros::Time::now();
  ik_time = (end_time - start_time).toSec();
  start_time = end_time;

  // Gravity compensation
  // https://answers.ros.org/question/286904/get-arm-gravity-compensation-joint-efforts-from-urdf-joint-positions-and-grav-vector/
  kdl_chain_dyn_param_->JntToGravity(q_, jnt_gravity_);
  for (int i = 0; i < joints_.size(); i++)
  {
    tau_(i) = tau_(i) + jnt_gravity_(i);
  }
  
  end_time = ros::Time::now();
  gravity_time = (end_time - start_time).toSec();
  start_time = end_time;
  
  // Clamp torque (I think gazebo is just ignoring commands that are too large)
  for (int i = 0; i < joints_.size(); i++)
  {
    tau_(i) = tau_(i) >  max_effort_[i] ?  max_effort_[i] : tau_(i);
    tau_(i) = tau_(i) < -max_effort_[i] ? -max_effort_[i] : tau_(i);
  }

  // And finally send these torques out
  for (int i = 0; i < joints_.size(); i++)
  {
    joints_[i].setCommand(tau_(i));
  }

  //publish the current state
  if (!(updates_ % 10)) {
    if (controller_state_publisher_ && 
        controller_state_publisher_->trylock()) {
      controller_state_publisher_->msg_.header.stamp = last_time_;
      controller_state_publisher_->msg_.desired = 
        desiredPose;
      controller_state_publisher_->msg_.actual_pose.pose.position.x = 
        x_.p.x();
      controller_state_publisher_->msg_.actual_pose.pose.position.y = 
        x_.p.y();
      controller_state_publisher_->msg_.actual_pose.pose.position.z = 
        x_.p.z();
      x_.M.GetQuaternion
        (controller_state_publisher_->msg_.actual_pose.pose.orientation.x,
         controller_state_publisher_->msg_.actual_pose.pose.orientation.y,
         controller_state_publisher_->msg_.actual_pose.pose.orientation.z,
         controller_state_publisher_->msg_.actual_pose.pose.orientation.w);
      controller_state_publisher_->msg_.actual_pose.
        wrench_or_stiffness.force.x = F_(0);
      controller_state_publisher_->msg_.actual_pose.
        wrench_or_stiffness.force.y = F_(1);
      controller_state_publisher_->msg_.actual_pose.
        wrench_or_stiffness.force.z = F_(2);
      controller_state_publisher_->msg_.actual_pose.
        wrench_or_stiffness.torque.x = F_(3);
      controller_state_publisher_->msg_.actual_pose.
        wrench_or_stiffness.torque.y = F_(4);
      controller_state_publisher_->msg_.actual_pose.
        wrench_or_stiffness.torque.z = F_(5);

      // read the actual current effort
      // this could be pulled into the loop below, but who cares.
      for (int i = 0; i < joints_.size(); i++)
      {
        tau_act_(i) = joints_[i].getEffort();
      }

      double eff_err = 0;
      for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
        eff_err += (tau_(i) - tau_act_(i))*(tau_(i) - tau_act_(i));
        controller_state_publisher_->msg_.requested_joint_efforts[i] = 
          tau_(i);
        controller_state_publisher_->msg_.actual_joint_efforts[i] = 
          tau_act_(i);
      }
      controller_state_publisher_->msg_.effort_sq_error = eff_err;
      boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
      desired_poses_box_.get(desired_poses_ptr);
      controller_state_publisher_->msg_.goal = desired_poses_ptr->traj;
      controller_state_publisher_->msg_.initial_point = last_point_;
      controller_state_publisher_->msg_.running_time =
        time - desired_poses_ptr->starting_time;
      controller_state_publisher_->unlockAndPublish();
    }
  }
  updates_++;
  end_time = ros::Time::now();
  after_time = (end_time - start_time).toSec();
  
  ROS_DEBUG_THROTTLE(1, "Total Time Guess: %f; Prepare: %f; Their IK: %f; IK: %f; Gravity: %f; After: %f",
     (end_time - very_start_time).toSec(), prepare_time, their_ik_time, ik_time, gravity_time, after_time); 
}

void EECartImpedControlClass::starting(const ros::Time& time) {
  hold_current_pose(time);
}

void EECartImpedControlClass::stopping(const ros::Time& time) {
  hold_current_pose(time);
}

// COPIED FROM pr2_mechanism_model::Chain
// fills out kdl_chain_ variable and joints_ variable
bool EECartImpedControlClass::constructKDLChain(std::string root, std::string tip, std::string robot_desc_string) {
  // Constructs the kdl chain
  KDL::Tree kdl_tree;
  bool waitLoop = true;
  while (waitLoop) {
    ros::Duration(0.5).sleep();
    ros::param::get("/WaitLoop", waitLoop);
  }
  // kdl_parser from http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
  if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
    ROS_ERROR("Could not convert robot_description string into kdl tree");
    return false;
  }

  bool res;
  try{
    res = kdl_tree.getChain(root, tip, kdl_chain_);
  }
  catch(...){
    res = false;
  }
  if (!res){
    ROS_ERROR("Could not extract chain between %s and %s from kdl tree",
        root.c_str(), tip.c_str());
    return false;
  }

  // Pulls out all the JointHandles and save them to our vector
  joints_.clear();
  for (size_t i=0; i<kdl_chain_.getNrOfSegments(); i++){
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None){ 
      hardware_interface::JointHandle jnt = hardware_interface_->getHandle(kdl_chain_.getSegment(i).getJoint().getName());
      joints_.push_back(jnt);
    }
  }
  ROS_DEBUG("Added %i joints", int(joints_.size()));

  return true;
}

/// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(ee_cart_imped_control_ns::EECartImpedControlClass, controller_interface::ControllerBase)
