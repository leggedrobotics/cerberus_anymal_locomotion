#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/JointState.h>
#include <cerberus_anymal_control/utils/GraphLoader.hpp>
#include <cerberus_anymal_control/controllers/cpgController_200205.hpp>
#include <cerberus_anymal_control/utils/IK.hpp>
#include <tf/transform_listener.h>

class CerberusAnymalControl
{

public:

  CerberusAnymalControl();
  ~CerberusAnymalControl();
  void Update();
  void CommandVelCallback(const geometry_msgs::Twist& twistMsg);
  void DirectJointStateCallback(const sensor_msgs::JointState& jointStateMsg);
  void IgnitionGazeboBodyVelocityCallback(const geometry_msgs::Twist& bodyVelocityMsg);
  void LoadParams();

  ros::NodeHandle nh_;

  std::unique_ptr<RAI::QuadrupedController> quadrupedController_;

private:

  ros::Subscriber cmd_vel_subscriber_;

  ros::Subscriber direct_joint_state_subscriber_;

  ros::Subscriber ignition_gazebo_body_velocity_subscriber_;

  ros::Publisher des_joint_position_publisher_;


  GraphLoader<float> graph_;
  Eigen::Matrix<double, 19, 1> genCoordinates_;
  Eigen::Matrix<double, 18, 1> genVelocities_;
  double xCom_, yCom_, zCom_;
  bool gotFirstJointPositionMsg_, gotFirstBodyPoseMsg_, gotFirstBodyVelocityMsg_;
  mav_msgs::Actuators desJointPositions_;

  int downSamplingCounterJointState_;

  // Looking up transform
  tf::TransformListener bodyPoseTFListener_;
  tf::StampedTransform bodyPoseTransform_;

  // Derivative for velocities
  ros::Time tfLookupTimeTMinusOne_;
  Eigen::Matrix<double, 3, 1> genBodyPositionsTMinusOne_;
  double bodyRoll_, bodyPitch_, bodyYaw_;
  double bodyRollTMinusOne_, bodyPitchTMinusOne_, bodyYawTMinusOne_;

  // String for parametrizing the topic name
  std::string robot_name_;
  std::string model_ns_;
};