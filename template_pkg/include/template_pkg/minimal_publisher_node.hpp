// Copyright 2024 Seaward Science
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#pragma once

// Include order: C++ standard library first, then other packages.
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// This is where we include all our namespace stuff for the package
#include "template_pkg/package_defs.hpp"

NS_HEAD  // macro for consistently defining our namespace for the package

/**
 * @brief a minimal publisher class that extends the rclcpp::Node class to
 * demonstrate how to create ROS2 Node
 *
 * This class is used to create a ROS2 publisher that regularly publishes
 * messages to a topic.
 */
class MinimalPublisherNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   * The constructor initializes the publisher and timer.
   */
  MinimalPublisherNode();

  /**
   * @brief Destructor for the MinimalPublisherNode class.
   */
  ~MinimalPublisherNode();

  /**
   * @brief Structure for holding parameters for the MinimalPublisherNode class.
   */
  struct Parameters
  {
    int timer_period = 500;      ///< Timer period in milliseconds

    /**
     * @brief User-friendly ROS QoS settings for a topic endpoint.
     *
     * The string values are intended to be set from a ROS parameter YAML
     * file. Supported reliability values are `best_effort`, `reliable`,
     * and `system_default`; durability supports `volatile`,
     * `transient_local`, and `system_default`; history supports
     * `keep_last` and `keep_all`.
     */
    struct QosSettings
    {
      std::string reliability = "best_effort";
      std::string durability = "volatile";
      std::string history = "keep_last";
      int depth = 10;

      void declare(MinimalPublisherNode * node, const std::string & prefix);
      void update(MinimalPublisherNode * node, const std::string & prefix);
      rclcpp::QoS make_qos() const;
    };

    struct Topics
    {
      std::string subscriber_topic = "input_topic";        ///< Name of the input topic
      std::string publisher_topic = "output_topic";        ///< Name of the output topic
    } topics;

    QosSettings subscriber_qos;      ///< QoS used by the input subscription
    QosSettings publisher_qos;      ///< QoS used by the output publisher

    Parameters();
    void declare(MinimalPublisherNode * node);
    void update(MinimalPublisherNode * node);
  };

  /**
   * @brief Structure for holding subscriptions for the MinimalPublisherNode class.
   */
  struct Subscribers
  {
    /// Shared pointer to the subscription
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void init(MinimalPublisherNode * node);
  };

  /**
   * @brief Structure for holding publishers for the MinimalPublisherNode class.
   */
  struct Publishers
  {
    /// Shared pointer to the publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    void init(MinimalPublisherNode * node);
  };

  /**
   * @brief Structure for holding diagnostics components for the MinimalPublisherNode class.
   *
   * Uses diagnostic_updater to periodically publish diagnostic status and
   * monitor the publication frequency of the output topic.
   */
  struct Diagnostics
  {
    /// Shared pointer to the diagnostic updater
    std::shared_ptr<diagnostic_updater::Updater> updater;
    void init(MinimalPublisherNode * node);
  };

protected:
  /**
   * @brief Callback function for the timer
   *
   * This function is called periodically by the timer. It publishes
   * a message containing a string with a counter.
   */
  void timer_callback();

  /**
   * @brief Callback function for the subscription
   *
   * This function is called when a message is received on the subscribed topic.
   *
   * @param msg Shared pointer to the received message.
   */
  void subscription_callback(std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Diagnostic callback for reporting node status.
   *
   * This function is registered with the diagnostic updater and called
   * periodically to report the health and state of the node.
   *
   * @param stat Wrapper for building the diagnostic status message.
   */
  void check_node_status(diagnostic_updater::DiagnosticStatusWrapper & stat);

  Parameters parameters_;
  Subscribers subscribers_;
  Publishers publishers_;
  Diagnostics diagnostics_;

  std::string message_;
  rclcpp::TimerBase::SharedPtr timer_;    ///< Shared pointer to the timer
  size_t count_;    ///< Counter for the number of messages published
};

NS_FOOT
