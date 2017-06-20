/**
 *  @file   problem.h
 *  @brief  problem: load, create, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef PROBLEM_H_
#define PROBLEM_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <gtsam/geometry/Pose2.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/utils/fileUtils.h>

#include <robot.h>
#include <misc.h>


namespace piper {

class Problem
{
  public:
    Robot robot;
    gtsam::Vector start_conf, goal_conf;
    gtsam::Pose2 start_pose, goal_pose;
    gpmp2::Pose2Vector pstart, pgoal;
    gpmp2::SignedDistanceField sdf;
    double total_time, cost_sigma, epsilon, delta_t;
    int total_step, obs_check_inter, control_inter;
    gpmp2::TrajOptimizerSetting opt_setting;
    
  private:
    std::vector<double> sc_, gc_, sp_, gp_;
    std::string sdf_file_;
    double Qc_, fix_pose_sigma_, fix_vel_sigma_;
    std::string opt_type_;

  public:
    /// Default constructor
    Problem() {}

    /**
     *  Loads problem from yaml file
     *
     *  @param nh node handle for namespace
     **/
    Problem(ros::NodeHandle nh);

    /// Default destructor
    virtual ~Problem() {}
};

} // piper namespace

#endif
