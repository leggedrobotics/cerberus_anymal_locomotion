/*!
 * @name      CerberusAnymalControlBaseRos.hpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 1, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#pragma once

// cerberus_anymal_control_base
#include "cerberus_anymal_control_base/CerberusAnymalControlBase.hpp"

// cerberus_anymal_control_ros
#include "cerberus_anymal_control_ros/CerberusAnymalControlRos.hpp"

namespace cerberus_anymal_control_ros {

template <typename ConcreteQuadrupedController>
class CerberusAnymalControlBaseRos : public cerberus_anymal_control::CerberusAnymalControlBase<ConcreteQuadrupedController>,
                                     public CerberusAnymalControlRos
{
 public:
  using Controller = cerberus_anymal_control::CerberusAnymalControlBase<ConcreteQuadrupedController>;
  using RosInterface = cerberus_anymal_control_ros::CerberusAnymalControlRos;

  CerberusAnymalControlBaseRos(ros::NodeHandle &nodeHandle,const int numberOfJoints);
  ~CerberusAnymalControlBaseRos() = default;

  void advanceRos() override;

 private:
  void commandVelCallback(const geometry_msgs::Twist &twistMsg) override;
  void jointStateCallback(const sensor_msgs::JointState &jointStateMsg) override;
  void bodyTwistCallback(const geometry_msgs::Twist &bodyTwistMsg) override;
};

} // namespace cerberus_anymal_control_ros

#include "cerberus_anymal_control_base/CerberusAnymalControlBaseRos.tpp"
