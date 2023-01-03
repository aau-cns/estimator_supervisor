// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>

#include <ros/ros.h>
#include <string>
#include "estimator_supervisor.h"

// Main function
int main(int argc, char** argv)
{
  // Launch ros node
  ros::init(argc, argv, "estimator_supervisor");
  ros::NodeHandle nh("~");

  ROS_INFO("Starting the Estimator Supervisor");

  // Start asynch (multi-threading) spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Define parameters
  double window_s, max_norm;
  std::string topic, msg_type;

  // Parse parameters
  if (!nh.getParam("supervisor_window_s", window_s))
  {
    std::cout << std::endl;
    ROS_ERROR("No Supervisor window defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("max_norm_changes", max_norm))
  {
    std::cout << std::endl;
    ROS_ERROR("No Maximum norm for changes after initialization defined");
    std::exit(EXIT_FAILURE);
  }

  if (!nh.getParam("topic_to_supervise", topic))
  {
    std::cout << std::endl;
    ROS_ERROR("No topic to supervise defined");
    std::exit(EXIT_FAILURE);
  }
  else
  {
    ROS_INFO("Supervising estimate from: %s", topic.c_str());
  }

  if (!nh.getParam("estiamte_msg_type", msg_type))
  {
    std::cout << std::endl;
    ROS_ERROR("No message type for topic to supervised defined");
    std::exit(EXIT_FAILURE);
  }
  else
  {
    ROS_INFO("Supervising estimate with message type: %s", topic.c_str());
  }

  // Instanciate the supervisor
  Supervisor Supervisor(nh, window_s, max_norm, topic, msg_type);

  // Wait for shutdown
  ros::waitForShutdown();

  // Stop the spinner
  spinner.stop();

  // Done!
  std::cout << std::endl;
  ROS_INFO("Done!");
  return EXIT_SUCCESS;
}
