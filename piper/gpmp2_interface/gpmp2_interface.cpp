/**
 *  @file   gpmp2_interface.cpp
 *  @brief  ROS interface between GPMP2 and a real/sim robot
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#include <gpmp2_interface.h>


namespace piper {

/* ************************************************************************** */
GPMP2Interface::GPMP2Interface(ros::NodeHandle nh)
{
  // first load problem and setup trajectory client
  problem_ = Problem(nh);
  traj_ = Traj(nh);

  // robot state subscriber (used to initialize start state if not passed as param)
  if (nh.hasParam("robot/arm_state_topic"))
  {
    nh.getParam("robot/arm_state_topic", arm_state_topic_);
    arm_state_sub_ = nh.subscribe(arm_state_topic_, 1, &GPMP2Interface::armStateCallback, this);
    arm_pos_ = gtsam::Vector::Zero(problem_.robot.getDOFarm());
    arm_pos_time_ = ros::Time::now();
  }
  if (problem_.robot.isMobileBase() && nh.hasParam("robot/base_state_topic"))
  {
    nh.getParam("robot/base_state_topic", base_state_topic_);
    base_state_sub_ = nh.subscribe(base_state_topic_, 1, &GPMP2Interface::baseStateCallback, this);
    base_pos_ = gtsam::Pose2();
    base_pos_time_ = ros::Time::now();
  }
  ros::Duration(1.0).sleep();

  // get start from measurement if not passed as param
  if (!nh.hasParam("start_conf"))
  {
    problem_.start_conf = arm_pos_;
    if (problem_.robot.isThetaNeg())
      problem_.robot.negateTheta(problem_.start_conf);
  }
  if (problem_.robot.isMobileBase())
  {
    if (!nh.hasParam("start_pose"))
      problem_.start_pose = base_pos_;
    problem_.pstart = gpmp2::Pose2Vector(problem_.start_pose, problem_.start_conf);
  }
  
  // initialize trajectory
  traj_.initializeTrajectory(init_values_, problem_);

  // solve with batch gpmp2
  ROS_INFO("Optimizing...");
  int DOF = problem_.robot.getDOF();
  if (!problem_.robot.isMobileBase())
    batch_values_ = gpmp2::BatchTrajOptimize3DArm(problem_.robot.arm, problem_.sdf, problem_.start_conf, 
      gtsam::Vector::Zero(DOF), problem_.goal_conf, gtsam::Vector::Zero(DOF), init_values_, problem_.opt_setting);
  else
    batch_values_ = gpmp2::BatchTrajOptimizePose2MobileArm(problem_.robot.marm, problem_.sdf, problem_.pstart, 
      gtsam::Vector::Zero(DOF), problem_.pgoal, gtsam::Vector::Zero(DOF), init_values_, problem_.opt_setting);
  ROS_INFO("Batch GPMP2 optimization complete.");

  // publish trajectory for visualization or other use
  if (traj_.plan_traj_pub)
    traj_.publishPlannedTrajectory(batch_values_, problem_, 0);
}

/* ************************************************************************** */
void GPMP2Interface::execute()
{
  size_t exec_step;
  double coll_cost;

  // interpolate batch solution to a desired resolution for control and check for collision
  ROS_INFO("Checking for collision.");
  if (!problem_.robot.isMobileBase())
  {
    exec_values_ = gpmp2::interpolateArmTraj(batch_values_, problem_.opt_setting.Qc_model, problem_.delta_t, 
      problem_.control_inter, 0, problem_.total_step-1);
    coll_cost = gpmp2::CollisionCost3DArm(problem_.robot.arm, problem_.sdf, exec_values_, problem_.opt_setting);
  }
  else
  {
    exec_values_ = gpmp2::interpolatePose2MobileArmTraj(batch_values_, problem_.opt_setting.Qc_model, 
      problem_.delta_t, problem_.control_inter, 0, problem_.total_step-1);
    coll_cost = gpmp2::CollisionCostPose2MobileArm(problem_.robot.marm, problem_.sdf, exec_values_, problem_.opt_setting);
  }
  if (coll_cost != 0)
  {
    ROS_FATAL("Plan is not collision free! Collision cost = %.3f", coll_cost);
    sigintHandler(0);
  }

  //  execute trajectory
  ROS_INFO("Executing GPMP2 planned trajectory open-loop...");
  exec_step = problem_.total_step+problem_.control_inter*(problem_.total_step-1);
  traj_.executeTrajectory(exec_values_, problem_, exec_step);
}

/* ************************************************************************** */
void GPMP2Interface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  size_t index;
  for (size_t i=0; i<problem_.robot.getDOFarm(); i++)
  {
    index = std::distance(msg->name.begin(), find(msg->name.begin(), msg->name.end(), 
      traj_.arm_joint_names[i]));
    arm_pos_[i] = msg->position[index];
  }
  arm_pos_time_ = ros::Time::now();
}

/* ************************************************************************** */
void GPMP2Interface::baseStateCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  base_pos_ = gtsam::Pose2(msg->position.x, msg->position.y, gtsam::Rot3::Quaternion(msg->orientation.w, 
    msg->orientation.x, msg->orientation.y, msg->orientation.z).yaw());
  base_pos_time_ = ros::Time::now();
}

} // piper namespace


/* ************************************************************************** */
/* main callback */
void mainCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ros::NodeHandle nh("piper");
  piper::GPMP2Interface gpmp2(nh);
  gpmp2.execute();
  ROS_INFO("Done.");
  ros::shutdown();
}

/* ************************************************************************** */
/* main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gpmp2_interface");
  signal(SIGINT, piper::sigintHandler);
  ros::MultiThreadedSpinner spinner(0);

  ros::NodeHandle n;
  ros::Publisher main_pub = n.advertise<std_msgs::Bool>("/piper/run_main", 1);
  ros::Subscriber main_sub = n.subscribe("/piper/run_main", 1, mainCallback);
  main_pub.publish(std_msgs::Bool());

  spinner.spin();
}
