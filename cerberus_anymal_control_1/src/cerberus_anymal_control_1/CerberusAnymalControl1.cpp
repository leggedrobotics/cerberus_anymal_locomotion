/*!
 * @name      CerberusAnymalControl1.cpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 1, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

// ROS
#include <ros/package.h>

// cerberus_anymal_control_1
#include "cerberus_anymal_control_1/CerberusAnymalControl1.hpp"

namespace cerberus_anymal_control_1 {

CerberusAnymalControl1::CerberusAnymalControl1(ros::NodeHandle &nodeHandle) :
  RosController(nodeHandle,12)
{
  loadParameters();
};

void CerberusAnymalControl1::initialize() {
  ROS_INFO_STREAM("[CerberusAnymalControl1::initialize] Initializing controller.");
  quadrupedController_->reset(&graph_);
  ROS_INFO_STREAM("[CerberusAnymalControl1::initialize] Done!");
}

void CerberusAnymalControl1::advanceRos() {
  RosController::advanceRos();
}

void CerberusAnymalControl1::loadParameters() {
  ROS_INFO_STREAM("[CerberusAnymalControl1::loadParameters] loading params...");
  std::string package_path = ros::package::getPath("cerberus_anymal_control_1");
  package_path += "/config/200205";
  RosController::loadParameters(package_path);
  ROS_INFO_STREAM("[CerberusAnymalControl1::loadParameters] Done!");
}

} // namespace cerberus_anymal_control_1
