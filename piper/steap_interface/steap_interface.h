/**
 *  @file   steap_interface.h
 *  @brief  ROS interface between STEAP and a real/sim robot
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef STEAP_INTERFACE_H_
#define STEAP_INTERFACE_H_

#include <map>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/ISAM2TrajOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>

#include <problem.h>
#include <traj.h>
#include <misc.h>


namespace piper {

class STEAPInterface
{
  private:
    Problem problem_;
    Traj traj_;
    gtsam::Values init_values_, batch_values_, inc_inf_values_, exec_values_;
    gpmp2::ISAM2TrajOptimizer3DArm arm_inc_inf_;
    gpmp2::ISAM2TrajOptimizerPose2MobileArm marm_inc_inf_;

    std::string arm_state_topic_, base_state_topic_;
    ros::Subscriber arm_state_sub_, base_state_sub_;
    gtsam::Vector arm_pos_;
    gtsam::Pose2 base_pos_;
    ros::Time arm_pos_time_, base_pos_time_;

  public:
    /// Default constructor
    STEAPInterface() {}

    /**
     *  STEAP: simultaneous trajectory estimation and planning
     *
     *  @param nh node handle for namespace
     **/
    STEAPInterface(ros::NodeHandle nh);

    /// Default destructor
    virtual ~STEAPInterface() {}

    /**
     *  closed loop online execution
     **/
    void execute();

    /**
     *  Call back to get current state of the arm
     *
     *  @param msg message from arm state subscriber
     **/
    void armStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /**
     *  Call back to get current state of the base
     *
     *  @param msg message from base state subscriber
     **/
    void baseStateCallback(const geometry_msgs::Pose::ConstPtr& msg);
};

}

#endif
