/**
 *  @file   robot.h
 *  @brief  robot: load, create, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Pose2MobileArmModel.h>

#include <misc.h>


namespace piper {

class Robot
{
  public:
    gpmp2::ArmModel arm;
    gpmp2::Pose2MobileArmModel marm;
    double sensor_arm_sigma, sensor_base_sigma;
 
  private:
    bool mobile_base_;
    int DOF_, DOF_arm_;
    std::vector<double> a_, alpha_, d_, theta_;
    std::vector<bool> theta_neg_;
    std::vector<double> orientation_, position_;
    gtsam::Pose3 arm_base_;
    std::vector<double> js_, xs_, ys_, zs_, rs_;
    gpmp2::BodySphereVector spheres_data_;    

  public:
    /// Default constructor
    Robot() {}

    /**
     *  Constructor loads robot parameters from yaml file and constructs a robot
     *
     *  @param nh node handle for namespace
     **/
    Robot(ros::NodeHandle nh);

    /// Default destructor
    virtual ~Robot() {}

    /// check to see if robot has a mobile base
    inline bool isMobileBase() const { return mobile_base_; }

    /// get DOF for full body robot
    inline int getDOF() const { return DOF_; }

    /// get DOF for just the arm
    inline int getDOFarm() const { return DOF_arm_; }

    /// theta offset in DH params has a sign flip
    inline bool isThetaNeg() const { return !theta_neg_.empty(); }

    /**
     *  Flip sign of theta in DH parameters
     *
     *  @param conf change conf to account for theta bias
     **/
    void negateTheta(gtsam::Vector& conf);
};

} // piper namespace

#endif
