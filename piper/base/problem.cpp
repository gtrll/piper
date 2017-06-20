/**
 *  @file   problem.cpp
 *  @brief  problem: load, create, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#include <problem.h>


namespace piper {

/* ************************************************************************** */
Problem::Problem(ros::NodeHandle nh)
{
  // first load robot
  robot = Robot(nh);
  
  // given the robot load the planning problem
  ROS_INFO("Loading planning problem.");
  
  // start and goal
  if (nh.hasParam("start_conf"))
  {
    nh.getParam("start_conf", sc_);
    start_conf = getVector(sc_);
    if (robot.isThetaNeg())
      robot.negateTheta(start_conf);
  }
  nh.getParam("goal_conf", gc_);
  goal_conf = getVector(gc_);
  if (robot.isThetaNeg())
    robot.negateTheta(goal_conf);
  if (robot.isMobileBase())
  {
    if (nh.hasParam("start_pose"))
    {
      nh.getParam("start_pose", sp_);
      start_pose = gtsam::Pose2(sp_[0], sp_[1], sp_[2]);
    }
    nh.getParam("goal_pose", gp_);
    goal_pose = gtsam::Pose2(gp_[0], gp_[1], gp_[2]);
    pgoal = gpmp2::Pose2Vector(goal_pose, goal_conf);
  }

  // signed distance field
  nh.getParam("sdf_file", sdf_file_);
  sdf_file_ = ros::package::getPath("piper") + "/" + sdf_file_;
  std::string fext = sdf_file_.substr(sdf_file_.find_last_of(".") + 1);
  if (fext == "vol")
    gpmp2::readSDFvolfile(sdf_file_, sdf);
  else
    sdf.loadSDF(sdf_file_);
  
  // optimization settings
  nh.getParam("total_time", total_time);
  nh.getParam("total_step", total_step);
  nh.getParam("obs_check_inter", obs_check_inter);
  nh.getParam("control_inter", control_inter);
  nh.getParam("cost_sigma", cost_sigma);
  nh.getParam("epsilon", epsilon);
  nh.getParam("opt_type", opt_type_);
  nh.getParam("Qc", Qc_);
  nh.getParam("fix_pose_sigma", fix_pose_sigma_);
  nh.getParam("fix_vel_sigma", fix_vel_sigma_);
  int DOF = robot.getDOF();
  opt_setting = gpmp2::TrajOptimizerSetting(DOF);
  opt_setting.total_time = total_time;
  opt_setting.total_step = total_step-1;
  delta_t = total_time/(total_step-1);
  opt_setting.obs_check_inter = obs_check_inter;
  opt_setting.cost_sigma = cost_sigma;
  opt_setting.epsilon = epsilon;
  opt_setting.Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc_*gtsam::Matrix::Identity(DOF, DOF));
  opt_setting.conf_prior_model = gtsam::noiseModel::Isotropic::Sigma(DOF, fix_pose_sigma_);
  opt_setting.vel_prior_model = gtsam::noiseModel::Isotropic::Sigma(DOF, fix_vel_sigma_);
  if (opt_type_ == "LM")
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::LM;
  else if (opt_type_ == "Dogleg")
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::Dogleg;
  else if (opt_type_ == "GaussNewton")
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::GaussNewton;
  else
  {
    ROS_ERROR("Optimization type \'%s\' not known!", opt_type_.c_str());
    sigintHandler(0);
  }
}

} // piper namespace
