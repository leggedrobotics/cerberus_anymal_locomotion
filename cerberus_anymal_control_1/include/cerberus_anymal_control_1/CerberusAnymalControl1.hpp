/*!
 * @name      CerberusAnymalControl1.hpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 1, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#pragma once

// controller
#include "cerberus_anymal_control_1/controller/cpgController_200205.hpp"

// ros controller base class
#include "cerberus_anymal_control_base/CerberusAnymalControlBaseRos.hpp"

namespace cerberus_anymal_control_1 {

class CerberusAnymalControl1 : public cerberus_anymal_control_ros::CerberusAnymalControlBaseRos<RAI::QuadrupedController> {
 public:
  using RosController = cerberus_anymal_control_ros::CerberusAnymalControlBaseRos<RAI::QuadrupedController>;

  explicit CerberusAnymalControl1(ros::NodeHandle &nodeHandle);
  ~CerberusAnymalControl1() = default;

  void advanceRos() override;

 private:
  void loadParameters();
  void initialize() override;
};

} // namespace cerberus_anymal_control_1