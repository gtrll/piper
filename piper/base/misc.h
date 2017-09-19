/**
 *  @file   misc.h
 *  @brief  miscellaneous functions
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef MISC_H_
#define MISC_H_

#include <signal.h>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <gtsam/base/Vector.h>


namespace piper {

/**
 *  Convert std double vector type to gtsam Vector
 *
 *  @param v std double vector
 **/
static const gtsam::Vector getVector(const std::vector<double>& v)
{
  gtsam::Vector send(v.size());
  for (size_t i=0; i<v.size(); i++)
    send[i] = v[i];
  return send;
}

/**
 *  CTRL+C handling
 **/
static void sigintHandler(int sig)
{
  ROS_FATAL("Quitting...");
  ros::shutdown();
}


} // piper namespace

#endif
