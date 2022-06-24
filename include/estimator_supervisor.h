// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <Eigen/Eigen>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>
#include <vector>

#include "utils/buffer.h"

class Supervisor {

public:
  /**
   * @brief Autonomy constructor
   * @param Ros NodeHandle
   * @param Ros Time window
   * @param Ros maximun norm to accept changes as non diverging estimator
   * @param Topic to supervise
   * @param Message type
   */
  Supervisor(ros::NodeHandle &nh, double &window, double &max_norm,
             std::string &topic, std::string &msg_type);

private:
  /**
   * @brief Estimate with covariance callback
   * @param constant pointer to the message
   */
  void estimateWithCovarianceCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief Estimate with covariance callback
   * @param constant pointer to the message
   */
  void estimateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Service handler (handle the request of supervision)
   * @param Request
   * @param Responce
   */
  bool serviceHandler(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res);

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
  bool buffer_full_ = false;
};

#endif // SUPERVISOR_H
