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

#include "estimator_supervisor.h"
#include "utils/colors.h"

Supervisor::Supervisor(ros::NodeHandle& nh, double& window, double& max_norm, std::string& topic, std::string& msg_type)
  : nh_(nh), window_(window), max_norm_(max_norm), topic_(topic), msg_type_(msg_type)
{
  // Advertise supervisor service
  srv_ = nh_.advertiseService("service/supervision", &Supervisor::serviceHandler, this);
}

bool Supervisor::serviceHandler(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  std::cout << YELLOW("Supervisor service called") << std::endl;

  // Clear buffer
  position_buffer_.clear();
  buffer_full_ = false;

  // Subscribe to estimator and buffer data
  if (msg_type_ == "posestamped")
  {
    sub_estimate_ = nh_.subscribe(topic_, 1, &Supervisor::estimateCallback, this);
  }
  else if (msg_type_ == "posewithcovariancestamped")
  {
    sub_estimate_ = nh_.subscribe(topic_, 1, &Supervisor::estimateWithCovarianceCallback, this);
  }

  // Wait till the window gets filled by measurements
  while (!buffer_full_)
  {
    std::cout << YELLOW("Waiting for filling the estimate buffer...") << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Define response by calling estimator supervision
  res.success = estimatorSupervision();

  // Unsubscribe to avoid filling the buffer
  sub_estimate_.shutdown();

  // Success
  return true;
}

bool Supervisor::estimatorSupervision()
{
  // Safety check
  if (position_buffer_.front().timestamp <= 0.0 || position_buffer_.back().timestamp <= 0.0)
  {
    std::cout << RED("Buffer corrupted: first or last measurement has invalid "
                     "timestamp")
              << std::endl;
    return false;
  }

  // Compare the norm of difference between the
  // first and last element of the buffer
  if ((position_buffer_.back().p - position_buffer_.front().p).norm() > max_norm_)
  {
    return false;
  }

  return true;
}

void Supervisor::estimateWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Define local data structure
  Buffer::positionBuffer data;

  // Parse message to defined data structure
  data.timestamp = msg->header.stamp.toSec();
  data.p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  fillPositionBuffer(data);
}

void Supervisor::estimateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Define local data structure
  Buffer::positionBuffer data;

  // Parse message to defined data structure
  data.timestamp = msg->header.stamp.toSec();
  data.p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  fillPositionBuffer(data);
}
