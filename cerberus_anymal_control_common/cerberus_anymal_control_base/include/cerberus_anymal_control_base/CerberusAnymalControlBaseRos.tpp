/*!
 * @name      CerberusAnymalControlBaseRos.tpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 1, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

// Eigen
#include <eigen_conversions/eigen_msg.h>

#include "cerberus_anymal_control_base/CerberusAnymalControlBaseRos.hpp"

namespace cerberus_anymal_control_ros {

template<typename ConcreteQuadrupedController>
CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::CerberusAnymalControlBaseRos(ros::NodeHandle &nodeHandle) :
    Controller(),
    RosInterface(nodeHandle)
{
  desJointPositionMsg_.angles.resize(cerberus_anymal_control::numberOfJoints);
};

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::advanceRos() {
  // Calculate the control commands
  Controller::advance();
  if (Controller::initializedGraph_) {
    // Write the control commands
    boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
    for (int jointIterator = 0; jointIterator < cerberus_anymal_control::numberOfJoints; jointIterator++) {
      desJointPositionMsg_.angles[jointIterator] = Controller::desJointPositions_(jointIterator);
    }
    // Publish the control commands
    RosInterface::advanceRos();
  }
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::commandVelCallback(const geometry_msgs::Twist &twistMsg) {
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  // Twist
  tf::twistMsgToEigen(twistMsg,Controller::desTwist_);
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::jointStateCallback(const sensor_msgs::JointState &jointStateMsg) {
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  mapJointState(jointStateMsg,Controller::genCoordinates_,Controller::genVelocities_);
  // Set the flag
  Controller::hasJointPositionData_ = true;
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::bodyTwistCallback(const geometry_msgs::Twist &bodyTwistMsg) {
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  // Set the body twist
  mapBodyTwist(bodyTwistMsg,Controller::genVelocities_);
  // Set the flag
  Controller::hasBodyVelocityData_ = true;
}

// Helper Functions
template<typename ConcreteQuadrupedController>
inline void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::mapJointState(
    const sensor_msgs::JointState &jointStateMsg,
    cerberus_anymal_control::GenCoordinates &genCoordinates,
    cerberus_anymal_control::GenVelocities &genVelocities) {
  // LF
  genCoordinates(7) = jointStateMsg.position[0];
  genVelocities(6) = jointStateMsg.velocity[0];
  genCoordinates(8) = jointStateMsg.position[1];
  genVelocities(7) = jointStateMsg.velocity[1];
  genCoordinates(9) = jointStateMsg.position[2];
  genVelocities(8) = jointStateMsg.velocity[2];
  // LH
  genCoordinates(13) = jointStateMsg.position[3];
  genVelocities(12) = jointStateMsg.velocity[3];
  genCoordinates(14) = jointStateMsg.position[4];
  genVelocities(13) = jointStateMsg.velocity[4];
  genCoordinates(15) = jointStateMsg.position[5];
  genVelocities(14) = jointStateMsg.velocity[5];
  // RF
  genCoordinates(10) = jointStateMsg.position[6];
  genVelocities(9) = jointStateMsg.velocity[6];
  genCoordinates(11) = jointStateMsg.position[7];
  genVelocities(10) = jointStateMsg.velocity[7];
  genCoordinates(12) = jointStateMsg.position[8];
  genVelocities(11) = jointStateMsg.velocity[8];
  // RH
  genCoordinates(16) = jointStateMsg.position[9];
  genVelocities(15) = jointStateMsg.velocity[9];
  genCoordinates(17) = jointStateMsg.position[10];
  genVelocities(16) = jointStateMsg.velocity[10];
  genCoordinates(18) = jointStateMsg.position[11];
  genVelocities(17) = jointStateMsg.velocity[11];
}

template<typename ConcreteQuadrupedController>
inline void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::mapBodyTwist(
    const geometry_msgs::Twist &bodyTwistMsg,
    cerberus_anymal_control::GenVelocities &genVelocities) {
  genVelocities(0) = bodyTwistMsg.linear.x;
  genVelocities(1) = bodyTwistMsg.linear.y;
  genVelocities(2) = bodyTwistMsg.linear.z;
  genVelocities(3) = bodyTwistMsg.angular.x;
  genVelocities(4) = bodyTwistMsg.angular.y;
  genVelocities(5) = bodyTwistMsg.angular.z;
}

} // namespace cerberus_anymal_control_ros
