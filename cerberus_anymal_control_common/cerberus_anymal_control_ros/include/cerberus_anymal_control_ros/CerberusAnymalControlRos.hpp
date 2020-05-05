/*!
 * @name      CerberusAnymalControlRos.hpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 1, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/JointState.h>

namespace cerberus_anymal_control_ros {

class CerberusAnymalControlRos {
 public:
  explicit CerberusAnymalControlRos(ros::NodeHandle &nodeHandle);
  ~CerberusAnymalControlRos() = default;

 private:
  void publishRos();

 protected:
  void loadParametersRos();
  void initializeRos();
  virtual void advanceRos();

  // Callback methods
  virtual void commandVelCallback(const geometry_msgs::Twist &twistMsg) = 0;
  virtual void jointStateCallback(const sensor_msgs::JointState &jointStateMsg) = 0;
  virtual void bodyTwistCallback(const geometry_msgs::Twist &bodyTwistMsg) = 0;
  virtual void bodyPoseCallback(const geometry_msgs::TransformStamped &bodyPose) = 0;

  // Node handle
  ros::NodeHandle& nodeHandle_;

  // Subscriber
  ros::Subscriber commandVelSubscriber_;
  std::string commandVelSubscriberTopic_;

  ros::Subscriber jointStateSubscriber_;
  std::string jointStateSubscriberTopic_;

  ros::Subscriber bodyTwistSubscriber_;
  std::string bodyTwistSubscriberTopic_;

  ros::Subscriber bodyPoseSubscriber_;
  std::string bodyPoseSubscriberTopic_;

  // Publisher
  ros::Publisher desJointPositionPublisher_;
  std::string desJointPositionPublisherTopic_;
  mav_msgs::Actuators desJointPositionMsg_;
};

} // namespace cerberus_anymal_control_ros



