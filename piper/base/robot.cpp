/**
 *  @file   robot.h
 *  @brief  robot: load, create, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#include <robot.h>


namespace piper {

/* ************************************************************************** */
Robot::Robot(ros::NodeHandle nh)
{
  // get parameters for desired robot
  ROS_INFO("Loading robot parameters.");
  nh.getParam("robot/DOF", DOF_);
  if (nh.hasParam("robot/mobile_base"))
    nh.getParam("robot/mobile_base", mobile_base_);
  else
    mobile_base_ = false;
  if (nh.hasParam("robot/arm_base/orientation"))
    nh.getParam("robot/arm_base/orientation", orientation_); // quaternion: [x, y, z, w]
  else
    orientation_ = (std::vector<double>){0, 0, 0, 1};
  if (nh.hasParam("robot/arm_base/position"))
    nh.getParam("robot/arm_base/position", position_); // [x, y, z]
  else
    position_ = (std::vector<double>){0, 0, 0};
  nh.getParam("robot/DH/a", a_);
  nh.getParam("robot/DH/alpha", alpha_);
  nh.getParam("robot/DH/d", d_);
  nh.getParam("robot/DH/theta", theta_);
  if (nh.hasParam("robot/DH/theta_neg"))
    nh.getParam("robot/DH/theta_neg", theta_neg_);
  nh.getParam("robot/spheres/js", js_);
  nh.getParam("robot/spheres/xs", xs_);
  nh.getParam("robot/spheres/ys", ys_);
  nh.getParam("robot/spheres/zs", zs_);
  nh.getParam("robot/spheres/rs", rs_);
  if (nh.hasParam("robot/sensor_arm_sigma"))
    nh.getParam("robot/sensor_arm_sigma", sensor_arm_sigma);
  else
    sensor_arm_sigma = 0.0001;
  if (nh.hasParam("robot/sensor_base_sigma"))
    nh.getParam("robot/sensor_base_sigma", sensor_base_sigma);
  else
    sensor_base_sigma = 0.0001;

  // arm's base pose (relative to robot base if mobile_base_ is true)
  arm_base_ = gtsam::Pose3(gtsam::Rot3::Quaternion(orientation_[3], orientation_[0], orientation_[1], 
    orientation_[2]), gtsam::Point3(getVector(position_)));
  
  // spheres to approximate robot body: js - link id, rs - radius, [xs, ys, zs] - center
  for (size_t i=0; i<js_.size(); i++)
    spheres_data_.push_back(gpmp2::BodySphere(js_[i], rs_[i], gtsam::Point3(xs_[i], ys_[i], zs_[i])));
  
  // generate arm/mobile arm
  if (!mobile_base_)
  {
    DOF_arm_ = DOF_;
    arm = gpmp2::ArmModel(gpmp2::Arm(DOF_arm_, getVector(a_), getVector(alpha_), getVector(d_), arm_base_, getVector(theta_)), 
      spheres_data_);
  }
  else
  {
    DOF_arm_ = DOF_-3;
    marm = gpmp2::Pose2MobileArmModel(gpmp2::Pose2MobileArm(gpmp2::Arm(DOF_arm_, getVector(a_), getVector(alpha_), getVector(d_), 
      gtsam::Pose3(), getVector(theta_)), arm_base_), spheres_data_);
  }
}

/* ************************************************************************** */
void Robot::negateTheta(gtsam::Vector& conf)
{
  for (size_t i=0; i<conf.size(); i++)
    if (theta_neg_[i])
      conf[i] *= -1.0;
}

} // piper namespace
