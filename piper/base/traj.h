/**
 *  @file   traj.h
 *  @brief  trajectory: action client, initialize, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef TRAJ_H_
#define TRAJ_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gpmp2/geometry/Pose2Vector.h>

#include <problem.h>
#include <misc.h>


namespace piper {

class Traj
{
  public:
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
    ros::Publisher est_traj_pub, plan_traj_pub;
    std::vector<std::string> arm_joint_names;

  private:
    std::string trajectory_control_topic_, est_traj_pub_topic_, plan_traj_pub_topic_;
    TrajClient* traj_client_;
    control_msgs::FollowJointTrajectoryGoal traj_;

  public:
    /// Default constructor
    Traj() {}

    /**
     *  setup traj client and load params
     *
     *  @param nh node handle for namespace
     **/
    Traj(ros::NodeHandle nh);

    /// Default destructor
    virtual ~Traj() {}
    
    /**
     *  initialize trajectory for optimization
     *
     *  @param init_values initialized traj save in to this variable
     *  @param problem all problem params and settings
     **/
    void initializeTrajectory(gtsam::Values& init_values, Problem& problem);
    
    /**
     *  initialize trajectory for optimization
     *
     *  @param exec_values optimized, interpolated, and collision checked traj to execute
     *  @param problem all problem params and settings
     *  @param exec_step number of points on the trajectory
     **/
    void executeTrajectory(gtsam::Values& exec_values, Problem& problem, size_t exec_step);

    /**
     *  publish estimated trajectory
     *
     *  @param values estimated part of traj
     *  @param problem all problem params and settings
     *  @param step estimated traj is from 0 to step
     **/
    void publishEstimatedTrajectory(gtsam::Values& values, Problem& problem, size_t step);

    /**
     *  publish planned trajectory
     *
     *  @param values planned part of traj
     *  @param problem all problem params and settings
     *  @param step planned traj is from step to total_step
     **/
    void publishPlannedTrajectory(gtsam::Values& values, Problem& problem, size_t step);
};

} // piper namespace

#endif
