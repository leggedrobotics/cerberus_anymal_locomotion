/*!
 * @name      CerberusAnymalCControl1.cpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 4, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

// ROS
#include <ros/package.h>

// cerberus_anymal_c_control_1
#include "cerberus_anymal_c_control_1/CerberusAnymalCControl1.hpp"

namespace cerberus_anymal_c_control_1 {

CerberusAnymalCControl1::CerberusAnymalCControl1(ros::NodeHandle &nodeHandle) :
    RosController(nodeHandle)
{
  loadParameters();
};

void CerberusAnymalCControl1::initialize() {
  ROS_INFO_STREAM("[CerberusAnymalCControl1::initialize] Initializing controller.");
  quadrupedController_->reset(&graph_, genCoordinates_, genVelocities_);
  ROS_INFO_STREAM("[CerberusAnymalCControl1::initialize] Done!");
}

void CerberusAnymalCControl1::advanceRos() {
  RosController::advanceRos();
}

void CerberusAnymalCControl1::loadParameters() {
  ROS_INFO_STREAM("[CerberusAnymalCControl1::loadParameters] loading params...");
  std::string package_path = ros::package::getPath("cerberus_anymal_c_control_1");
  package_path += "/config/20200628";
  RosController::loadParameters(package_path);
  ROS_INFO_STREAM("[CerberusAnymalCControl1::loadParameters] Done!");
}

} // namespace cerberus_c_anymal_control_1
