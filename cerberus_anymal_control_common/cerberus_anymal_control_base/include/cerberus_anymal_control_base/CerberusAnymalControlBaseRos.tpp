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

#include "cerberus_anymal_control_base/CerberusAnymalControlBaseRos.hpp"

namespace cerberus_anymal_control_ros {

template<typename ConcreteQuadrupedController>
CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::CerberusAnymalControlBaseRos(ros::NodeHandle &nodeHandle,const int numberOfJoints) :
    Controller(numberOfJoints),
    RosInterface(nodeHandle)
{
  desJointPositionMsg_.angles.resize(numberOfJoints);
};

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::advanceRos() {
  // Calculate the control commands
  Controller::advance();
  if (Controller::initializedGraph_) {
    // Write the control commands
    boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
    // LF (HAA, HFE, KFE)
    desJointPositionMsg_.angles[0] = Controller::desJointPositions_(0);
    desJointPositionMsg_.angles[1] = Controller::desJointPositions_(1);
    desJointPositionMsg_.angles[2] = Controller::desJointPositions_(2);
    // RF (HAA, HFE, KFE)
    desJointPositionMsg_.angles[3] = Controller::desJointPositions_(3);
    desJointPositionMsg_.angles[4] = Controller::desJointPositions_(4);
    desJointPositionMsg_.angles[5] = Controller::desJointPositions_(5);
    // LH (HAA, HFE, KFE)
    desJointPositionMsg_.angles[6] = Controller::desJointPositions_(6);
    desJointPositionMsg_.angles[7] = Controller::desJointPositions_(7);
    desJointPositionMsg_.angles[8] = Controller::desJointPositions_(8);
    // RH (HAA, HFE, KFE)
    desJointPositionMsg_.angles[9] = Controller::desJointPositions_(9);
    desJointPositionMsg_.angles[10] = Controller::desJointPositions_(10);
    desJointPositionMsg_.angles[11] = Controller::desJointPositions_(11);
    // Publish the control commands
    RosInterface::advanceRos();
  }
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::commandVelCallback(const geometry_msgs::Twist &twistMsg) {
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  // Linear Velocity
  Controller::desLinearVelocity_[0] = twistMsg.linear.x;
  Controller::desLinearVelocity_[1] = twistMsg.linear.y;
  Controller::desLinearVelocity_[2] = twistMsg.linear.z;
  // Angualr Velocity
  Controller::desAngularVelocity_[0] = twistMsg.angular.x;
  Controller::desAngularVelocity_[1] = twistMsg.angular.y;
  Controller::desAngularVelocity_[2] = twistMsg.angular.z;
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::jointStateCallback(const sensor_msgs::JointState &jointStateMsg) {
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  // LF
  Controller::genCoordinates_(7) = jointStateMsg.position[0];
  Controller::genVelocities_(6) = jointStateMsg.velocity[0];
  Controller::genCoordinates_(8) = jointStateMsg.position[1];
  Controller::genVelocities_(7) = jointStateMsg.velocity[1];
  Controller::genCoordinates_(9) = jointStateMsg.position[2];
  Controller::genVelocities_(8) = jointStateMsg.velocity[2];
  // LH
  Controller::genCoordinates_(13) = jointStateMsg.position[3];
  Controller::genVelocities_(12) = jointStateMsg.velocity[3];
  Controller::genCoordinates_(14) = jointStateMsg.position[4];
  Controller::genVelocities_(13) = jointStateMsg.velocity[4];
  Controller::genCoordinates_(15) = jointStateMsg.position[5];
  Controller::genVelocities_(14) = jointStateMsg.velocity[5];
  // RF
  Controller::genCoordinates_(10) = jointStateMsg.position[6];
  Controller::genVelocities_(9) = jointStateMsg.velocity[6];
  Controller::genCoordinates_(11) = jointStateMsg.position[7];
  Controller::genVelocities_(10) = jointStateMsg.velocity[7];
  Controller::genCoordinates_(12) = jointStateMsg.position[8];
  Controller::genVelocities_(11) = jointStateMsg.velocity[8];
  // RH
  Controller::genCoordinates_(16) = jointStateMsg.position[9];
  Controller::genVelocities_(15) = jointStateMsg.velocity[9];
  Controller::genCoordinates_(17) = jointStateMsg.position[10];
  Controller::genVelocities_(16) = jointStateMsg.velocity[10];
  Controller::genCoordinates_(18) = jointStateMsg.position[11];
  Controller::genVelocities_(17) = jointStateMsg.velocity[11];
  // Set the flag
  if(!Controller::hasJointPositionData_) {
    Controller::hasJointPositionData_ = true;
  }
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::bodyTwistCallback(const geometry_msgs::Twist &bodyTwistMsg) {
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  // Set the body twist
  Controller::genVelocities_(0) = bodyTwistMsg.linear.x;
  Controller::genVelocities_(1) = bodyTwistMsg.linear.y;
  Controller::genVelocities_(2) = bodyTwistMsg.linear.z;
  Controller::genVelocities_(3) = bodyTwistMsg.angular.x;
  Controller::genVelocities_(4) = bodyTwistMsg.angular.y;
  Controller::genVelocities_(5) = bodyTwistMsg.angular.z;
  // Set the flag
  if(!Controller::hasBodyVelocityData_) {
    Controller::hasBodyVelocityData_ = true;
  }
}

template<typename ConcreteQuadrupedController>
void CerberusAnymalControlBaseRos<ConcreteQuadrupedController>::bodyPoseCallback(const geometry_msgs::TransformStamped &bodyPose) {
  using namespace cerberus_anymal_control;
  boost::shared_lock<boost::shared_mutex>lock(Controller::data_mutex_);
  // Set the body pose
  Controller::genCoordinates_(0) = bodyPose.transform.translation.x;
  Controller::genCoordinates_(1) = bodyPose.transform.translation.y;
  Controller::genCoordinates_(2) = bodyPose.transform.translation.z;
  Controller::genCoordinates_(3) = bodyPose.transform.rotation.w;
  Controller::genCoordinates_(4) = bodyPose.transform.rotation.x;
  Controller::genCoordinates_(5) = bodyPose.transform.rotation.y;
  Controller::genCoordinates_(6) = bodyPose.transform.rotation.z;
  // Set the flag
  if(!Controller::hasBodyPoseData_) {
    Controller::hasBodyPoseData_ = true;
  }
}

} // namespace cerberus_anymal_control_ros