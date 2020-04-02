// Ros controller for Darpa SubT virtual competition

#include <ros/ros.h>
#include <ros/package.h>
#include <cerberus_anymal_control/CerberusAnymalControl.hpp>


CerberusAnymalControl::CerberusAnymalControl()
{

  nh_.getParam("CerberusAnymalControl/robot_name", robot_name_);

  model_ns_ = "/" + robot_name_;
  
  // Subscribers:
  std::string commandVelocityTopic = model_ns_ + "/cmd_vel";
  cmd_vel_subscriber_ = nh_.subscribe(commandVelocityTopic, 1000, &CerberusAnymalControl::CommandVelCallback, this);

  std::string jointStateTopic = model_ns_ + "/joint_state";
  direct_joint_state_subscriber_ = nh_.subscribe(jointStateTopic, 1000, &CerberusAnymalControl::DirectJointStateCallback, this);

  std::string robotBaseTwistTopic = model_ns_ + "/robot_base_twist";
  ignition_gazebo_body_velocity_subscriber_ = nh_.subscribe(robotBaseTwistTopic, 1000, &CerberusAnymalControl::IgnitionGazeboBodyVelocityCallback, this);

  // Publisher:
  std::string desiredJointPosTopic = model_ns_ + "/desired_joint_positions";
  des_joint_position_publisher_ = nh_.advertise<mav_msgs::Actuators>(desiredJointPosTopic, 1000);

  // Learning Controller setup:
  quadrupedController_.reset(new RAI::QuadrupedController());
  LoadParams();

  xCom_ = 0.0;
  yCom_ = 0.0;
  zCom_ = 0.0;
  gotFirstJointPositionMsg_ = gotFirstBodyPoseMsg_ = gotFirstBodyVelocityMsg_ = false;

  desJointPositions_.angles.resize(12);
  downSamplingCounterJointState_ = 0;
}

CerberusAnymalControl::~CerberusAnymalControl(){};

void CerberusAnymalControl::Update()
{

  if (gotFirstJointPositionMsg_ && gotFirstBodyPoseMsg_ && gotFirstBodyVelocityMsg_) {

    Eigen::Matrix<double, 12, 1> jointPositions;
    quadrupedController_->getDesJointPos(jointPositions,
                                         genCoordinates_,
                                         genVelocities_,
                                         xCom_,
                                         yCom_,
                                         zCom_);

    //LF (HAA, HFE, KFE)
    desJointPositions_.angles[0] = jointPositions(0);
    desJointPositions_.angles[1] = jointPositions(1);
    desJointPositions_.angles[2] = jointPositions(2);
    //RF (HAA, HFE, KFE)
    desJointPositions_.angles[3] = jointPositions(3);
    desJointPositions_.angles[4] = jointPositions(4);
    desJointPositions_.angles[5] = jointPositions(5);
    //LH (HAA, HFE, KFE)
    desJointPositions_.angles[6] = jointPositions(6);
    desJointPositions_.angles[7] = jointPositions(7);
    desJointPositions_.angles[8] = jointPositions(8);
    //RH (HAA, HFE, KFE)
    desJointPositions_.angles[9] = jointPositions(9);
    desJointPositions_.angles[10] = jointPositions(10);
    desJointPositions_.angles[11] = jointPositions(11);

    des_joint_position_publisher_.publish(desJointPositions_);
  }
}

void CerberusAnymalControl::LoadParams()
{

  std::string package_path = ros::package::getPath("cerberus_anymal_control");

  graph_.initialize(package_path + "/config/200205/graph.pb");
  graph_.loadLP(package_path + "/config/200205/distill_params_1000.txt");

  // Initialize controller parameters
  quadrupedController_->reset(&graph_);
}

void CerberusAnymalControl::CommandVelCallback(const geometry_msgs::Twist& twistMsg)
{

  xCom_ = twistMsg.linear.x;
  yCom_ = twistMsg.linear.y;
  zCom_ = twistMsg.angular.z;
}

void CerberusAnymalControl::DirectJointStateCallback(const sensor_msgs::JointState& jointStateMsg) 
{
  if (downSamplingCounterJointState_ >= 1) {

    downSamplingCounterJointState_ = 0;

    // LF
    genCoordinates_(7) = jointStateMsg.position[0];
    genVelocities_(6) = jointStateMsg.velocity[0];
    genCoordinates_(8) = jointStateMsg.position[1];
    genVelocities_(7) = jointStateMsg.velocity[1];
    genCoordinates_(9) = jointStateMsg.position[2];
    genVelocities_(8) = jointStateMsg.velocity[2];

    // LH
    genCoordinates_(13) = jointStateMsg.position[3];
    genVelocities_(12) = jointStateMsg.velocity[3];
    genCoordinates_(14) = jointStateMsg.position[4];
    genVelocities_(13) = jointStateMsg.velocity[4];
    genCoordinates_(15) = jointStateMsg.position[5];
    genVelocities_(14) = jointStateMsg.velocity[5];

    // RF
    genCoordinates_(10) = jointStateMsg.position[6];
    genVelocities_(9) = jointStateMsg.velocity[6];
    genCoordinates_(11) = jointStateMsg.position[7];
    genVelocities_(10) = jointStateMsg.velocity[7];
    genCoordinates_(12) = jointStateMsg.position[8];
    genVelocities_(11) = jointStateMsg.velocity[8];

    // RH
    genCoordinates_(16) = jointStateMsg.position[9];
    genVelocities_(15) = jointStateMsg.velocity[9];
    genCoordinates_(17) = jointStateMsg.position[10];
    genVelocities_(16) = jointStateMsg.velocity[10];
    genCoordinates_(18) = jointStateMsg.position[11];
    genVelocities_(17) = jointStateMsg.velocity[11];

    // Lookup tf for Body Position and Attitude
    try{
      bodyPoseTFListener_.lookupTransform(model_ns_ + "/base", 
	    model_ns_, ros::Time(0), bodyPoseTransform_);
    }
    catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
    }
	    
    // Set generalized Coordinates from transform info:
    genCoordinates_(0) = bodyPoseTransform_.getOrigin().x();
    genCoordinates_(1) = bodyPoseTransform_.getOrigin().y();
    genCoordinates_(2) = bodyPoseTransform_.getOrigin().z();
    genCoordinates_(3) = bodyPoseTransform_.getRotation().w();
    genCoordinates_(4) = bodyPoseTransform_.getRotation().x();
    genCoordinates_(5) = bodyPoseTransform_.getRotation().y();
    genCoordinates_(6) = bodyPoseTransform_.getRotation().z();

    if (!gotFirstJointPositionMsg_) gotFirstJointPositionMsg_ = true;
    if (!gotFirstBodyPoseMsg_) gotFirstBodyPoseMsg_ = true;
  }

  else downSamplingCounterJointState_++;
	
}

void CerberusAnymalControl::IgnitionGazeboBodyVelocityCallback(const geometry_msgs::Twist& bodyVelocityMsg)
{

    genVelocities_(0) = bodyVelocityMsg.linear.x;
    genVelocities_(1) = bodyVelocityMsg.linear.y;
    genVelocities_(2) = bodyVelocityMsg.linear.z;
    genVelocities_(3) = bodyVelocityMsg.angular.x;
    genVelocities_(4) = bodyVelocityMsg.angular.y;
    genVelocities_(5) = bodyVelocityMsg.angular.z;

    if (!gotFirstBodyVelocityMsg_) gotFirstBodyVelocityMsg_ = true;
}


int main(int argc, char **argv)
{
  // Initialize ros
  ros::init(argc, argv, "CerberusAnymalControl");

  // Create the controller
  CerberusAnymalControl controller;

  ros::Rate loop_rate(400);
  while (ros::ok())
  {
    controller.Update();
    //ROS_INFO("10 Hz\n");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
