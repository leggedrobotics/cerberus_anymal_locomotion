/*!
 * @name      CerberusAnymalControlBase.hpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      April 30, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#pragma once

// C++
#include <memory>

// Boost
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Core>

// Learning Controller
#include "cerberus_anymal_utils/GraphLoader.hpp"

namespace cerberus_anymal_control {

template <typename ConcreteQuadrupedController>
class CerberusAnymalControlBase {
 public:
  CerberusAnymalControlBase() = default;
  explicit CerberusAnymalControlBase(const unsigned int numberOfJoints);
  ~CerberusAnymalControlBase() = default;

 protected:
  virtual void initialize() = 0;
  virtual void loadParameters(const std::string& path);
  void advance();

  //! @brief Pointer to the specific learning controller
  std::unique_ptr<ConcreteQuadrupedController> quadrupedController_;

  //! @brief Graph for the learning controller
  GraphLoader<float> graph_;

 protected:
  const unsigned int numberOfJoints_;
  //! @brief Coordinates for the controller
  //! Position(x,y,z),Orientation(w,x,y,z),JointPositions(LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE)
  Eigen::Matrix<double, 19, 1> genCoordinates_;
  //! LinearVelocity(x,y,z),AngularVelocity(x,y,z),JointVelocities(LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE)
  Eigen::Matrix<double, 18, 1> genVelocities_;
  //! LinearVelocity(x,y,z), AngularVelocity(x,y,z)
  Eigen::Matrix<double, 6, 1> desTwist_;
  Eigen::VectorXd desJointPositions_;

  //! @brief Mutex for the controller data
  mutable boost::shared_mutex data_mutex_;

  //! @brief Checks that data has arrived
  bool hasBodyVelocityData_;
  bool hasJointPositionData_;
  bool initializedGraph_;
};

} // namespace cerberus_anymal_control

#include "CerberusAnymalControlBase.tpp"
