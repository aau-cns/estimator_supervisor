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

#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Eigen>
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "utils/buffer.h"

class Supervisor
{
public:
  /**
   * @brief Autonomy constructor
   * @param Ros NodeHandle
   * @param Ros Time window
   * @param Ros maximun norm to accept changes as non diverging estimator
   * @param Topic to supervise
   * @param Message type
   */
  Supervisor(ros::NodeHandle& nh, double& window, double& max_norm, std::string& topic, std::string& msg_type);

private:
  /**
   * @brief Estimate with covariance callback
   * @param constant pointer to the message
   */
  void estimateWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  /**
   * @brief Estimate with covariance callback
   * @param constant pointer to the message
   */
  void estimateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief fillPositionBuffer
   * @param data
   */
  inline void fillPositionBuffer(const Buffer::positionBuffer& data)
  {
    // Push data into buffer
    position_buffer_.emplace_back(data);

    // Remove oldest if sensor reading window width is reached
    if ((data.timestamp - position_buffer_.front().timestamp) > window_)
    {
      // Set reached window flag
      buffer_full_ = true;

      // Since we check everytime a new measurement is added to the buffer it is
      // sufficient to simply remove the first element
      position_buffer_.erase(position_buffer_.begin());
    }
  }

  /**
   * @brief Service handler (handle the request of supervision)
   * @param Request
   * @param Responce
   */
  bool serviceHandler(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Estimator supervision
   * @param Request
   * @param Responce
   */
  [[nodiscard]] bool estimatorSupervision();

  /// Nodehandler
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber sub_estimate_;

  /// Supervisor service
  ros::ServiceServer srv_;

  /// Window of time in seconds to check for changes of the state estimate
  double window_;

  /// Maximum norm to accept an estimate as non diverging
  double max_norm_;

  /// Topic where state estimation is published
  std::string topic_;

  /// Message type
  std::string msg_type_;

  /// Define vector of position measurement
  std::vector<Buffer::positionBuffer> position_buffer_;

  /// Boolean flag when buffer is full
  std::atomic<bool> buffer_full_ = false;
};

#endif  // SUPERVISOR_H
