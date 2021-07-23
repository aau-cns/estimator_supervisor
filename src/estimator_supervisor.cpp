// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
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

#include "estimator_supervisor.h"
#include "utils/colors.h"

Supervisor::Supervisor(ros::NodeHandle &nh, double &window, double &max_norm, std::string &topic) :
  nh_(nh), window_(window), max_norm_(max_norm), topic_(topic) {

  // Advertise supervisor service
  srv_ = nh_.advertiseService("service/supervision", &Supervisor::serviceHandler, this);

}

bool Supervisor::serviceHandler(std_srvs::Trigger::Request& , std_srvs::Trigger::Response& res) {

  std::cout << YELLOW("Supervisor service called") << std::endl;

  // Clear buffer
  position_buffer_.clear();
  buffer_full_ = false;

  // Subscribe to estimator and buffer data
  sub_pose_w_covariance_ = nh_.subscribe(topic_, 1, &Supervisor::estimateWithCovarianceCallback, this);

  // Wait till the window gets filled by measurements
  while (!buffer_full_) {
    std::cout << YELLOW("Waiting for filling the estimate buffer...") << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Define responce by calling estimator supervision
  res.success = estimatorSupervision();

  // Unsubscribe to avoid filling the buffer
  sub_pose_w_covariance_.shutdown();

  // Success
  return true;
}

bool Supervisor::estimatorSupervision() {

  // Compare the norm of  difference between the
  // first and last element of the buffer
  if ((position_buffer_.end()->p - position_buffer_.begin()->p).norm() > max_norm_) {
    return false;
  }

  return true;
}

void Supervisor::estimateWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

  // Define local data structure
  Buffer::positionBuffer data;

  // Parse message to defined data structure
  data.timestamp = msg->header.stamp.toSec();
  data.p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  // Push data into buffer
  position_buffer_.emplace_back(data);

  // Remove oldest if sensor reading window width is reached
  if ((data.timestamp - position_buffer_.begin()->timestamp) > window_) {

    // Set reached window flag
    buffer_full_ = true;

    // Since we check everytime a new measurement is added to the buffer it is
    // sufficient to simply remove the first element
    position_buffer_.erase(position_buffer_.begin());
  }
}
