// Copyright (c) 2024 Takumi Asada
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "pos_array_msgs/srv/set_position.hpp"
#include <chrono>

using SetPosition = pos_array_msgs::srv::SetPosition;
using namespace std::chrono_literals;

class SetPositionServer : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the SetPositionServer class.
   * Initializes the service node, creates a service, and sets up a timer for periodic actions.
   */
  SetPositionServer()
  : Node("set_position_server")
  {
    // Create a service that handles requests to "set_position"
    service_ = this->create_service<SetPosition>(
      "set_position",
      std::bind(&SetPositionServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    // Set up a timer to periodically execute the timer_callback function
    timer_ = this->create_wall_timer(2s, std::bind(&SetPositionServer::timer_callback, this));
  }

private:
  /**
   * @brief Callback function for handling service requests.
   * Logs the received request and sets the response based on the request data.
   *
   * @param request Pointer to the request object containing the service call parameters.
   * @param response Pointer to the response object where the result of the service call is set.
   */
  void handle_service(const std::shared_ptr<SetPosition::Request> request,
                      std::shared_ptr<SetPosition::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request: x=%f, y=%f, z=%f", request->x, request->y, request->z);

    // Simple processing: success if x > 0.0, otherwise failure
    response->success = (request->x > 0.0);

    RCLCPP_INFO(this->get_logger(), "Response: success=%d", response->success);
  }

  /**
   * @brief Timer callback function that runs periodically.
   * Logs a message indicating that the server is running.
   */
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Server is running...");
  }

  rclcpp::Service<SetPosition>::SharedPtr service_;  ///< Service for handling SetPosition requests
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic logging
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SetPositionServer>();

  // Spin the node to handle incoming service requests and periodic actions
  rclcpp::spin(node);  // Node can be stopped with Ctrl+C

  rclcpp::shutdown();
  return 0;
}
