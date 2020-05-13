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

#include "CerberusAnymalControlBase.hpp"

namespace cerberus_anymal_control {

template <typename ConcreteQuadrupedController>
CerberusAnymalControlBase<ConcreteQuadrupedController>::CerberusAnymalControlBase(int numberOfJoints) :
    quadrupedController_(),
    graph_(),
    numberOfJoints_(numberOfJoints),
    genCoordinates_(),
    genVelocities_(),
    desLinearVelocity_(Eigen::Vector3d::Zero()),
    desAngularVelocity_(Eigen::Vector3d::Zero()),
    desJointPositions_(Eigen::VectorXd::Zero(numberOfJoints)),
    hasBodyVelocityData_(false),
    hasJointPositionData_(false),
    initializedGraph_(false)
{
  quadrupedController_ = std::make_unique<ConcreteQuadrupedController>();
  if (numberOfJoints_ != 12) {
    std::cout << "[CerberusAnymalControlBase::CerberusAnymalControlBase] Joint Number is not equal to 12! Controller might not be working properly." << std::endl;
  }
  // Fix the pose since we already subscribe in base frame
  genCoordinates_(0) = 0.0;
  genCoordinates_(1) = 0.0;
  genCoordinates_(2) = 0.0;
  genCoordinates_(3) = 1.0;
  genCoordinates_(4) = 0.0;
  genCoordinates_(5) = 0.0;
  genCoordinates_(6) = 0.0;
}

template <typename ConcreteQuadrupedController>
void CerberusAnymalControlBase<ConcreteQuadrupedController>::loadParameters(const std::string& path) {
  std::cout << "[CerberusAnymalControlBase::loadParameters] Loading controller parameters" << std::endl;
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
      quadrupedController_->getDesJointPos(jointPositions, genCoordinates_, genVelocities_, desLinearVelocity_[0], desLinearVelocity_[1], desAngularVelocity_[2]);
    }
    // Iterate through all joints
    for (unsigned int jointIterator = 0; jointIterator < numberOfJoints_; jointIterator++) {
      desJointPositions_[jointIterator] = jointPositions(jointIterator);
    }
  }
}

} // namespace cerberus_anymal_control
