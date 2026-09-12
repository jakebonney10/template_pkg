#include "minimal_publisher_node.hpp"

#include <stdexcept>

using namespace std::chrono_literals;

NS_HEAD

MinimalPublisherNode::Parameters::Parameters(){} // default params are defined in header file

void MinimalPublisherNode::Parameters::QosSettings::declare(
    MinimalPublisherNode* node, const std::string& prefix)
{
    node->declare_parameter(prefix + ".reliability", reliability);
    node->declare_parameter(prefix + ".durability", durability);
    node->declare_parameter(prefix + ".history", history);
    node->declare_parameter(prefix + ".depth", depth);
}

void MinimalPublisherNode::Parameters::QosSettings::update(
    MinimalPublisherNode* node, const std::string& prefix)
{
    node->get_parameter(prefix + ".reliability", reliability);
    node->get_parameter(prefix + ".durability", durability);
    node->get_parameter(prefix + ".history", history);
    node->get_parameter(prefix + ".depth", depth);
}

rclcpp::QoS MinimalPublisherNode::Parameters::QosSettings::makeQos() const
{
    if (depth <= 0) {
        throw std::invalid_argument("QoS depth must be greater than zero");
    }

    rclcpp::QoS qos(static_cast<size_t>(depth));
    if (history == "keep_last") {
        qos.keep_last(static_cast<size_t>(depth));
    } else if (history == "keep_all") {
        qos.keep_all();
    } else {
        throw std::invalid_argument("Unsupported QoS history: " + history);
    }

    if (reliability == "best_effort") {
        qos.best_effort();
    } else if (reliability == "reliable") {
        qos.reliable();
    } else if (reliability == "system_default") {
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    } else {
        throw std::invalid_argument("Unsupported QoS reliability: " + reliability);
    }

    if (durability == "volatile") {
        qos.durability_volatile();
    } else if (durability == "transient_local") {
        qos.transient_local();
    } else if (durability == "system_default") {
        qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
    } else {
        throw std::invalid_argument("Unsupported QoS durability: " + durability);
    }

    return qos;
}

void MinimalPublisherNode::Parameters::declare(MinimalPublisherNode* node)
{
    node->declare_parameter("timer_period", timer_period);
    node->declare_parameter("topics.subscriber_topic", topics.subscriber_topic);
    node->declare_parameter("topics.publisher_topic", topics.publisher_topic);
    subscriber_qos.declare(node, "qos.subscriber");
    publisher_qos.declare(node, "qos.publisher");
}

void MinimalPublisherNode::Parameters::update(MinimalPublisherNode* node)
{
    node->get_parameter("timer_period", timer_period);
    node->get_parameter("topics.subscriber_topic", topics.subscriber_topic);
    node->get_parameter("topics.publisher_topic", topics.publisher_topic);
    subscriber_qos.update(node, "qos.subscriber");
    publisher_qos.update(node, "qos.publisher");
}

void MinimalPublisherNode::Subscribers::init(MinimalPublisherNode* node)
{
    subscription_ = node->create_subscription<std_msgs::msg::String>(
        node->parameters_.topics.subscriber_topic, node->parameters_.subscriber_qos.makeQos(),
        std::bind(&MinimalPublisherNode::subscriptionCallback, node, std::placeholders::_1));
}

void MinimalPublisherNode::Publishers::init(MinimalPublisherNode* node)
{
    publisher_ = node->create_publisher<std_msgs::msg::String>(
        node->parameters_.topics.publisher_topic, node->parameters_.publisher_qos.makeQos());
}

void MinimalPublisherNode::Diagnostics::init(MinimalPublisherNode* node)
{
    updater = std::make_shared<diagnostic_updater::Updater>(node);
    updater->setHardwareID("none");
    updater->add("Node Status", node, &MinimalPublisherNode::checkNodeStatus);
}

MinimalPublisherNode::MinimalPublisherNode()
    : Node("minimal_publisher"), count_(0)
{
    parameters_.declare(this);
    parameters_.update(this);
    subscribers_.init(this);
    publishers_.init(this);
    diagnostics_.init(this);

    message_ = "Hello, world! ";

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameters_.timer_period), std::bind(&MinimalPublisherNode::timer_callback, this));
}

void MinimalPublisherNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = message_ + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publishers_.publisher_->publish(message);
}

void MinimalPublisherNode::subscriptionCallback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    message_ = msg->data;
    // Try running `ros2 topic pub --once /input_topic std_msgs/msg/String "{data: 'Hello from command line'}"` from the command line
}

void MinimalPublisherNode::checkNodeStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    // Add key-value pairs to give context in the diagnostics viewer
    stat.add("Messages published", count_);

    // Use STALE when the diagnostic source has not produced data yet or has gone silent.
    // Use ERROR for critical failures where the node cannot function correctly.
    // Use WARN for degraded-but-recoverable conditions.
    // Use OK for normal operation.
    // Tip: set error/warn flags from other callbacks and check them here.
    if (count_ == 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No messages published yet");
    } else if (false /* replace: e.g. error_flag_ set from a failed operation */) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Describe the critical failure here");
    } else if (false /* replace: e.g. warn_flag_ set from degraded conditions */) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Describe the degraded condition here");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Publishing normally");
    }
}

MinimalPublisherNode::~MinimalPublisherNode() 
{
    RCLCPP_INFO(this->get_logger(), "MinimalPublisherNode destroyed");
}

NS_FOOT
