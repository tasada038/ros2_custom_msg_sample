#include "rclcpp/rclcpp.hpp"
#include "pos_array_msgs/srv/set_position.hpp"
#include <chrono>

using SetPosition = pos_array_msgs::srv::SetPosition;
using namespace std::chrono_literals;

class SetPositionClient : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the SetPositionClient class.
   * Initializes the client node and creates a service client.
   */
  SetPositionClient()
  : Node("set_position_client")
  {
    client_ = this->create_client<SetPosition>("set_position");
  }

  /**
   * @brief Calls the SetPosition service and waits for the response.
   * Sends a request to the SetPosition service with predefined x, y, z values
   * and waits for a response from the server.
   *
   * @return true if the service call is successful, false otherwise.
   */
  bool call_service()
  {
    // Check if the service is available
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for service...");
      return false;
    }

    // Create the request
    auto request = std::make_shared<SetPosition::Request>();
    request->x = 1.0;
    request->y = 2.0;
    request->z = 3.0;

    RCLCPP_INFO(this->get_logger(), "Sending request: x=%f, y=%f, z=%f", request->x, request->y, request->z);

    // Call the service asynchronously
    auto future = client_->async_send_request(request);

    // Wait for the response
    while (rclcpp::ok()) {
      // Spin the node to process incoming responses
      rclcpp::spin_some(this->get_node_base_interface());

      // Check if the response is ready
      if (future.wait_for(100ms) == std::future_status::ready) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Response received: success=%d", response->success);
        return true;
      }
    }
    return false;
  }

private:
  rclcpp::Client<SetPosition>::SharedPtr client_;  // Service client to communicate with the SetPosition service
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SetPositionClient>();

  // Call the service once from the main function
  if (node->call_service()) {
    RCLCPP_INFO(node->get_logger(), "Service call succeeded");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  // Shutdown the node after the service call
  rclcpp::shutdown();
  return 0;
}
