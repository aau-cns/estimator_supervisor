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

#include "estimator_supervisor.h"
#include <ros/ros.h>
#include <string>

// Main function
int main(int argc, char** argv) {

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
  if(!nh.getParam("Supervisor_window_s", window_s)) {
    std::cout << std::endl;
    ROS_ERROR("No Supervisor window defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("max_norm_changes", max_norm)) {
    std::cout << std::endl;
    ROS_ERROR("No Maximum norm for changes after initialization defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("topic_to_supervise", topic)) {
    std::cout << std::endl;
    ROS_ERROR("No topic to supervise defined");
    std::exit(EXIT_FAILURE);
  } else {
    ROS_INFO("Supervising estimate from: %s", topic.c_str());
  }

  if(!nh.getParam("estiamte_msg_type", msg_type)) {
    std::cout << std::endl;
    ROS_ERROR("No message type for topic to supervised defined");
    std::exit(EXIT_FAILURE);
  } else {
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
