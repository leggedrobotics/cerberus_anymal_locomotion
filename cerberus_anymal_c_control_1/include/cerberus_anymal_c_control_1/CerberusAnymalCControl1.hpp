/*!
 * @name      CerberusAnymalCControl1.hpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 4, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#pragma once

// controller
#include "cerberus_anymal_c_control_1/controller/cpgController_j.hpp"

// ros controller base class
#include "cerberus_anymal_control_base/CerberusAnymalControlBaseRos.hpp"

namespace cerberus_anymal_c_control_1 {

class CerberusAnymalCControl1 : public cerberus_anymal_control_ros::CerberusAnymalControlBaseRos<RAI::QuadrupedController> {
 public:
  using RosController = cerberus_anymal_control_ros::CerberusAnymalControlBaseRos<RAI::QuadrupedController>;

  explicit CerberusAnymalCControl1(ros::NodeHandle &nodeHandle);
  ~CerberusAnymalCControl1() = default;

  void advanceRos() override;

 private:
  void loadParameters();
  void initialize() override;
};

} // namespace cerberus_anymal_c_control_1
