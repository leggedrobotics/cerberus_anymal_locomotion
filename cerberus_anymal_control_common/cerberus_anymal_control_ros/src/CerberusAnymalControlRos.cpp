/*!
 * @name      CerberusAnymalControlRos.cpp
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

// cerberus_anymal_control_ros
#include "cerberus_anymal_control_ros/CerberusAnymalControlRos.hpp"

namespace cerberus_anymal_control_ros {

CerberusAnymalControlRos::CerberusAnymalControlRos(ros::NodeHandle &nodeHandle) :
  nodeHandle_(nodeHandle)
{
  loadParametersRos();
  initializeRos();
};

void CerberusAnymalControlRos::advanceRos() {
  publishRos();
}

void CerberusAnymalControlRos::initializeRos() {
  ROS_INFO("[CerberusAnymalControlRos::initializeRos] Initializing the ROS interface.");
  // Subscriber
  commandVelSubscriber_ = nodeHandle_.subscribe(commandVelSubscriberTopic_, 10, &CerberusAnymalControlRos::commandVelCallback,this);
  jointStateSubscriber_ = nodeHandle_.subscribe(jointStateSubscriberTopic_, 10, &CerberusAnymalControlRos::jointStateCallback, this);
  bodyTwistSubscriber_ = nodeHandle_.subscribe(bodyTwistSubscriberTopic_, 10, &CerberusAnymalControlRos::bodyTwistCallback, this);
  bodyPoseSubscriber_ = nodeHandle_.subscribe(bodyPoseSubscriberTopic_, 10, &CerberusAnymalControlRos::bodyPoseCallback, this);
  // Publisher
  desJointPositionPublisher_ = nodeHandle_.advertise<mav_msgs::Actuators>(desJointPositionPublisherTopic_, 10);
}

void CerberusAnymalControlRos::loadParametersRos() {
  ROS_INFO("[CerberusAnymalControlRos::loadParametersRos] Load Parameters.");
  bool success = true;
  // Load Parameters
  if(nodeHandle_.hasParam("subscriber/command_velocity/topic")) {
    nodeHandle_.getParam("subscriber/command_velocity/topic",commandVelSubscriberTopic_);
  } else {
    success = false;
    ROS_ERROR("[CerberusAnymalControlRos::loadParametersRos] Could not load command_velocity subscriber topic");
  }
  if(nodeHandle_.hasParam("subscriber/joint_state/topic")) {
    nodeHandle_.getParam("subscriber/joint_state/topic",jointStateSubscriberTopic_);
  } else {
    success = false;
    ROS_ERROR("[CerberusAnymalControlRos::loadParametersRos] Could not load joint_state subscriber topic");
  }
  if(nodeHandle_.hasParam("subscriber/body_twist/topic")) {
    nodeHandle_.getParam("subscriber/body_twist/topic",bodyTwistSubscriberTopic_);
  } else {
    success = false;
    ROS_ERROR("[CerberusAnymalControlRos::loadParametersRos] Could not load body_twist subscriber topic");
  }
  if(nodeHandle_.hasParam("subscriber/body_pose/topic")) {
    nodeHandle_.getParam("subscriber/body_pose/topic",bodyPoseSubscriberTopic_);
  } else {
    success = false;
    ROS_ERROR("[CerberusAnymalControlRos::loadParametersRos] Could not load body_pose subscriber topic");
  }
  if(nodeHandle_.hasParam("publisher/des_joint_position/topic")) {
    nodeHandle_.getParam("publisher/des_joint_position/topic",desJointPositionPublisherTopic_);
  } else {
    success = false;
    ROS_ERROR("[CerberusAnymalControlRos::loadParametersRos] Could not load des_joint_position publisher topic");
  }
  if(!success) {
    ROS_ERROR("[CerberusAnymalControlRos::loadParametersRos] Could not load parameters");
    ros::shutdown();
  }
}

void CerberusAnymalControlRos::publishRos() {
  desJointPositionPublisher_.publish(desJointPositionMsg_);
}

} // namespace cerberus_anymal_control_ros
