/*!
 * @name      CerberusAnymalControlBase.tpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      April 30, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

// ROS for printing
#include <ros/console.h>

#include "CerberusAnymalControlBase.hpp"

namespace cerberus_anymal_control {

template <typename ConcreteQuadrupedController>
CerberusAnymalControlBase<ConcreteQuadrupedController>::CerberusAnymalControlBase(const unsigned int numberOfJoints) :
    quadrupedController_(),
    graph_(),
    numberOfJoints_(numberOfJoints),
    genCoordinates_(),
    genVelocities_(),
    desTwist_(Eigen::VectorXd::Zero(6)),
    desJointPositions_(Eigen::VectorXd::Zero(numberOfJoints)),
    hasBodyVelocityData_(false),
    hasJointPositionData_(false),
    initializedGraph_(false)
{
  quadrupedController_ = std::make_unique<ConcreteQuadrupedController>();
  if (numberOfJoints_ != 12) {
    ROS_INFO("[CerberusAnymalControlBase::CerberusAnymalControlBase] Joint Number is not equal to 12! Controller might not be working properly.");
  }
  // Fix the pose since we already subscribe in base frame
  genCoordinates_ << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
}

template <typename ConcreteQuadrupedController>
void CerberusAnymalControlBase<ConcreteQuadrupedController>::loadParameters(const std::string& path) {
  ROS_INFO("[CerberusAnymalControlBase::loadParameters] Loading controller parameters");
  graph_.initialize(path + "/graph.pb");
  graph_.loadLP(path + "/params.txt");
}

template <typename ConcreteQuadrupedController>
void CerberusAnymalControlBase<ConcreteQuadrupedController>::advance() {
  if (hasBodyVelocityData_ && hasJointPositionData_) {
    if (!initializedGraph_) {
      initialize();
      initializedGraph_ = true;
    }
    Eigen::Matrix<double, 12, 1> jointPositions;
    {
      boost::unique_lock<boost::shared_mutex>lock(data_mutex_);
      quadrupedController_->getDesJointPos(jointPositions, genCoordinates_, genVelocities_, desTwist_(0), desTwist_(1), desTwist_(5));
    }
    // Iterate through all joints
    for (unsigned int jointIterator = 0; jointIterator < numberOfJoints_; jointIterator++) {
      desJointPositions_[jointIterator] = jointPositions(jointIterator);
    }
  }
}

} // namespace cerberus_anymal_control
