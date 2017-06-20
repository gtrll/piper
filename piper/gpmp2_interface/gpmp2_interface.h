/**
 *  @file   gpmp2_interface.h
 *  @brief  ROS interface between GPMP2 and a real/sim robot
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef GPMP2_INTERFACE_H_
#define GPMP2_INTERFACE_H_

#include <map>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>

#include <problem.h>
#include <traj.h>
#include <misc.h>


namespace piper {

class GPMP2Interface
{
  private:
    Problem problem_;
    Traj traj_;
    gtsam::Values init_values_, batch_values_, exec_values_;

    std::string arm_state_topic_, base_state_topic_;
    ros::Subscriber arm_state_sub_, base_state_sub_;
    gtsam::Vector arm_pos_;
    gtsam::Pose2 base_pos_;
    ros::Time arm_pos_time_, base_pos_time_;

  public:
    /// Default constructor
    GPMP2Interface() {}

    /**
     *  batch gpmp2
     *
     *  @param nh node handle for namespace
     **/
    GPMP2Interface(ros::NodeHandle nh);

    /// Default destructor
    virtual ~GPMP2Interface() {}

    /**
     *  Open-loop execution of GPMP2
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
