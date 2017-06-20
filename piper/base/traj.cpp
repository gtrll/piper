/**
 *  @file   traj.cpp
 *  @brief  trajectory: action client, initialize, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#include <traj.h>


namespace piper {

/* ************************************************************************** */
Traj::Traj(ros::NodeHandle nh)
{
  // ros trajectory joint names
  nh.getParam("robot/arm_joint_names", arm_joint_names);
  traj_.trajectory.joint_names = arm_joint_names;

  // to visualize estimated trajectory
  if (nh.hasParam("robot/est_traj_pub_topic"))
  {
    nh.getParam("robot/est_traj_pub_topic", est_traj_pub_topic_);
    est_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(est_traj_pub_topic_, 1);
  }

  // to visualize planned trajectory
  if (nh.hasParam("robot/plan_traj_pub_topic"))
  {
    nh.getParam("robot/plan_traj_pub_topic", plan_traj_pub_topic_);
    plan_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(plan_traj_pub_topic_, 1);
  }

  // trajectory action client
  if (nh.hasParam("robot/trajectory_control_topic"))
  {
    nh.getParam("robot/trajectory_control_topic", trajectory_control_topic_);
    traj_client_ = new Traj::TrajClient(trajectory_control_topic_, true);
    if (!traj_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for trajectory_control server...");
      if (!traj_client_->waitForServer(ros::Duration(5.0)))
      {
        ROS_ERROR("Cannot find trajectory_control server \'%s\'", trajectory_control_topic_.c_str());
        sigintHandler(0);
      }
    }
  }
  else
    ROS_WARN("No trajectory control topic. Trajectory will not be executed.");
}

/* ************************************************************************** */
void Traj::initializeTrajectory(gtsam::Values& init_values, Problem& problem)
{
  ROS_INFO("Initializing trajectory.");
  gtsam::Vector conf, avg_vel;
  if (!problem.robot.isMobileBase())
  {
    avg_vel = (problem.goal_conf - problem.start_conf)/problem.total_time;
    for (size_t i=0; i<problem.total_step; i++)
    {
      double ratio = static_cast<double>(i)/static_cast<double>(problem.total_step-1);
      conf = (1.0 - ratio)*problem.start_conf + ratio*problem.goal_conf;
      init_values.insert(gtsam::Symbol('x',i), conf);
      init_values.insert(gtsam::Symbol('v',i), avg_vel);
    }
  }
  else
  {
    gtsam::Pose2 pose;
    avg_vel = (gtsam::Vector(problem.robot.getDOF()) << problem.goal_pose.x()-problem.start_pose.x(), 
      problem.goal_pose.y()-problem.start_pose.y(), problem.goal_pose.theta()-problem.start_pose.theta(), 
      problem.goal_conf - problem.start_conf).finished()/problem.total_time;
    for (size_t i=0; i<problem.total_step; i++)
    {
      double ratio = static_cast<double>(i)/static_cast<double>(problem.total_step-1);
      pose = gtsam::Pose2((1.0 - ratio)*problem.start_pose.x() + ratio*problem.goal_pose.x(), 
        (1.0 - ratio)*problem.start_pose.y() + ratio*problem.goal_pose.y(), 
        (1.0 - ratio)*problem.start_pose.theta() + ratio*problem.goal_pose.theta());
      conf = (1.0 - ratio)*problem.start_conf + ratio*problem.goal_conf;
      init_values.insert(gtsam::Symbol('x',i), gpmp2::Pose2Vector(pose, conf));
      init_values.insert(gtsam::Symbol('v',i), avg_vel);
    }
  }
}

/* ************************************************************************** */
void Traj::executeTrajectory(gtsam::Values& exec_values, Problem& problem, size_t exec_step)
{
  gtsam::Pose2 pose;
  gtsam::Vector conf, vel;
  int DOF = problem.robot.getDOF();
  int DOF_arm = problem.robot.getDOFarm();

  // create ros trajectory
  traj_.trajectory.points.resize(exec_step);
  for (size_t i=0; i<exec_step; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      traj_.trajectory.points[i].positions.resize(DOF_arm);
      traj_.trajectory.points[i].velocities.resize(DOF_arm);      
      conf = exec_values.at<gtsam::Vector>(gtsam::Symbol('x',i));
      vel = exec_values.at<gtsam::Vector>(gtsam::Symbol('v',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (size_t j=0; j<DOF_arm; j++)
      {
        traj_.trajectory.points[i].positions[j] = conf[j];
        traj_.trajectory.points[i].velocities[j] = vel[j];
      }
    }
    else
    {
      traj_.trajectory.points[i].positions.resize(DOF);
      traj_.trajectory.points[i].velocities.resize(DOF);
      pose = exec_values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).pose();
      conf = exec_values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).configuration();
      vel = exec_values.at<gtsam::Vector>(gtsam::Symbol('v',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      traj_.trajectory.points[i].positions[0] = pose.x();
      traj_.trajectory.points[i].positions[1] = pose.y();
      traj_.trajectory.points[i].positions[2] = pose.theta();
      for (size_t j=0; j<DOF_arm; j++)
        traj_.trajectory.points[i].positions[j+3] = conf[j];
      for (size_t j=0; j<DOF; j++)
        traj_.trajectory.points[i].velocities[j] = vel[j];
    }
    traj_.trajectory.points[i].time_from_start = ros::Duration(i*problem.delta_t/(problem.control_inter+1));
  }
  traj_.trajectory.header.stamp = ros::Time::now();
    
  // dispatch ros trajectory
  traj_client_->sendGoal(traj_);
  traj_client_->waitForResult();
}

/* ************************************************************************** */
void Traj::publishEstimatedTrajectory(gtsam::Values& values, Problem& problem, size_t step)
{
  gtsam::Vector conf;
  gtsam::Pose2 pose;
  trajectory_msgs::JointTrajectory est_traj;
  est_traj.points.resize(step+1);
  for (size_t i=0; i<step+1; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      est_traj.points[i].positions.resize(problem.robot.getDOFarm());
      conf = values.at<gtsam::Vector>(gtsam::Symbol('x',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        est_traj.points[i].positions[j] = conf[j];
    }
    else
    {
      est_traj.points[i].positions.resize(problem.robot.getDOF());
      pose = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).pose();
      conf = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).configuration();
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      est_traj.points[i].positions[0] = pose.x();
      est_traj.points[i].positions[1] = pose.y();
      est_traj.points[i].positions[2] = pose.theta();
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        est_traj.points[i].positions[j+3] = conf[j];
    }
  }
  est_traj_pub.publish(est_traj);
}

/* ************************************************************************** */
void Traj::publishPlannedTrajectory(gtsam::Values& values, Problem& problem, size_t step)
{
  gtsam::Vector conf;
  gtsam::Pose2 pose;
  trajectory_msgs::JointTrajectory plan_traj;
  plan_traj.points.resize(problem.total_step-step);
  for (size_t i=step; i<problem.total_step; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      plan_traj.points[i-step].positions.resize(problem.robot.getDOFarm());
      conf = values.at<gtsam::Vector>(gtsam::Symbol('x',i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        plan_traj.points[i-step].positions[j] = conf[j];
    }
    else
    {
      plan_traj.points[i-step].positions.resize(problem.robot.getDOF());
      pose = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).pose();
      conf = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x',i)).configuration();
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      plan_traj.points[i-step].positions[0] = pose.x();
      plan_traj.points[i-step].positions[1] = pose.y();
      plan_traj.points[i-step].positions[2] = pose.theta();
      for (size_t j=0; j<problem.robot.getDOFarm(); j++)
        plan_traj.points[i-step].positions[j+3] = conf[j];
    }
  }
  plan_traj_pub.publish(plan_traj);
}

} // piper namespace
