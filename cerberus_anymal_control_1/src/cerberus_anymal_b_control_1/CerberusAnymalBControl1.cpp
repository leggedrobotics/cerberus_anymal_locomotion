/*!
 * @name      CerberusAnymalBControl1.cpp
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

// cerberus_anymal_b_control_1
#include "cerberus_anymal_b_control_1/CerberusAnymalBControl1.hpp"

namespace cerberus_anymal_b_control_1 {

CerberusAnymalBControl1::CerberusAnymalBControl1(ros::NodeHandle &nodeHandle) :
  RosController(nodeHandle,12)
{
  loadParameters();
};

void CerberusAnymalBControl1::initialize() {
  ROS_INFO_STREAM("[CerberusAnymalBControl1::initialize] Initializing controller.");
  quadrupedController_->reset(&graph_);
  ROS_INFO_STREAM("[CerberusAnymalBControl1::initialize] Done!");
}

void CerberusAnymalBControl1::advanceRos() {
  RosController::advanceRos();
}

void CerberusAnymalBControl1::loadParameters() {
  ROS_INFO_STREAM("[CerberusAnymalBControl1::loadParameters] loading params...");
  std::string package_path = ros::package::getPath("cerberus_anymal_b_control_1");
  package_path += "/config/200205";
  RosController::loadParameters(package_path);
  ROS_INFO_STREAM("[CerberusAnymalBControl1::loadParameters] Done!");
}

} // namespace cerberus_anymal_control_1
