/*!
 * @name      cerberus_anymal_c_control_1.cpp
 * @author    Samuel Zimmermann
 * @email     zsamuel@ethz.ch
 * @date      May 4, 2020
 *
 * Copyright (C) 2020 Robotic Systems Lab, ETH Zurich.
 * All rights reserved.
 * http://www.rsl.ethz.ch/
 */

// cerberus_anymal_c_control_1
#include "cerberus_anymal_c_control_1/CerberusAnymalCControl1.hpp"

int main(int argc, char** argv) {
  // Initialize ROS.
  ros::init(argc, argv, "cerberus_anymal_c_control_ros");
  ros::NodeHandle nodeHandle;

  // Create the controller
  cerberus_anymal_c_control_1::CerberusAnymalCControl1 controller(nodeHandle);

  // Set the controller frequency
  ros::Rate loop_rate(400);

  while (ros::ok())
  {
    // Run controller
    controller.advanceRos();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
