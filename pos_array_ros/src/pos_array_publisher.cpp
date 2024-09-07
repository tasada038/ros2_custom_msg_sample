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
#include "pos_array_msgs/msg/pos_array.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class PosArrayPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the PosArrayPublisher class.
   * Initializes the node, creates publishers for four different topics, and sets up timers to periodically publish messages.
   */
  PosArrayPublisher()
  : Node("pos_array_publisher")
  {
    // Create publishers for different topics
    publisher_1_ = this->create_publisher<pos_array_msgs::msg::PosArray>("pos_array_topic_1", 10);
    publisher_2_ = this->create_publisher<pos_array_msgs::msg::PosArray>("pos_array_topic_2", 10);
    publisher_3_ = this->create_publisher<pos_array_msgs::msg::PosArray>("pos_array_topic_3", 10);
    publisher_4_ = this->create_publisher<pos_array_msgs::msg::PosArray>("pos_array_topic_4", 10);

    // Set up timers to periodically call the publish_message function for each publisher
    timer_1_ = this->create_wall_timer(
      1s, [this]() { this->publish_message(publisher_1_, 1); });

    timer_2_ = this->create_wall_timer(
      1s, [this]() { this->publish_message(publisher_2_, 2); });

    timer_3_ = this->create_wall_timer(
      1s, [this]() { this->publish_message(publisher_3_, 3); });

    timer_4_ = this->create_wall_timer(
      1s, [this]() { this->publish_message(publisher_4_, 4); });
  }

private:
  /**
   * @brief Publishes a message with a specific ID to a given publisher.
   * Fills the message with data based on the ID and publishes it to the associated topic.
   *
   * @param publisher The publisher to send the message to.
   * @param id The ID to set in the message and determine the data content.
   */
  void publish_message(rclcpp::Publisher<pos_array_msgs::msg::PosArray>::SharedPtr publisher, int id)
  {
    auto message = pos_array_msgs::msg::PosArray();
    message.id = id;

    // Set different data based on the ID
    if (id == 1) {
      for (int i = 0; i < 3; ++i) {
        geometry_msgs::msg::Point point;
        point.x = static_cast<double>(i);
        point.y = static_cast<double>(i * 2);
        point.z = static_cast<double>(i * 3);
        message.positions.push_back(point);
      }
    } else if (id == 2) {
      for (int i = 0; i < 3; ++i) {
        geometry_msgs::msg::Point point;
        point.x = static_cast<double>(i * 2);
        point.y = static_cast<double>(i * 4);
        point.z = static_cast<double>(i * 6);
        message.positions.push_back(point);
      }
    } else if (id == 3) {
      for (int i = 0; i < 3; ++i) {
        geometry_msgs::msg::Point point;
        point.x = static_cast<double>(i * 3);
        point.y = static_cast<double>(i * 6);
        point.z = static_cast<double>(i * 9);
        message.positions.push_back(point);
      }
    } else if (id == 4) {
      for (int i = 0; i < 3; ++i) {
        geometry_msgs::msg::Point point;
        point.x = static_cast<double>(i * 4);
        point.y = static_cast<double>(i * 8);
        point.z = static_cast<double>(i * 12);
        message.positions.push_back(point);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Publishing ID: '%d' on topic '%s'", message.id, publisher->get_topic_name());
    for (const auto & pos : message.positions) {
      RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]", pos.x, pos.y, pos.z);
    }

    publisher->publish(message);
  }

  rclcpp::Publisher<pos_array_msgs::msg::PosArray>::SharedPtr publisher_1_; ///< Publisher for topic 1
  rclcpp::Publisher<pos_array_msgs::msg::PosArray>::SharedPtr publisher_2_; ///< Publisher for topic 2
  rclcpp::Publisher<pos_array_msgs::msg::PosArray>::SharedPtr publisher_3_; ///< Publisher for topic 3
  rclcpp::Publisher<pos_array_msgs::msg::PosArray>::SharedPtr publisher_4_; ///< Publisher for topic 4

  rclcpp::TimerBase::SharedPtr timer_1_; ///< Timer for periodically publishing messages on topic 1
  rclcpp::TimerBase::SharedPtr timer_2_; ///< Timer for periodically publishing messages on topic 2
  rclcpp::TimerBase::SharedPtr timer_3_; ///< Timer for periodically publishing messages on topic 3
  rclcpp::TimerBase::SharedPtr timer_4_; ///< Timer for periodically publishing messages on topic 4
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create and spin the PosArrayPublisher node
  rclcpp::spin(std::make_shared<PosArrayPublisher>());

  rclcpp::shutdown();
  return 0;
}
