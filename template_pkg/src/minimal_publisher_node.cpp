#include "minimal_publisher_node.hpp"

using namespace std::chrono_literals;

NS_HEAD

MinimalPublisherNode::Parameters::Parameters(){} // default params are defined in header file

void MinimalPublisherNode::Parameters::declare(MinimalPublisherNode* node)
{
    node->declare_parameter("timer_period", timer_period);
    node->declare_parameter("topics.subscriber_topic", topics.subscriber_topic);
    node->declare_parameter("topics.publisher_topic", topics.publisher_topic);
}

void MinimalPublisherNode::Parameters::update(MinimalPublisherNode* node)
{
    node->get_parameter("timer_period", timer_period);
    node->get_parameter("topics.subscriber_topic", topics.subscriber_topic);
    node->get_parameter("topics.publisher_topic", topics.publisher_topic);
}

void MinimalPublisherNode::Subscribers::init(MinimalPublisherNode* node)
{
    subscription_ = node->create_subscription<std_msgs::msg::String>(
        node->parameters_.topics.subscriber_topic, 10,
        std::bind(&MinimalPublisherNode::subscriptionCallback, node, std::placeholders::_1));
}

void MinimalPublisherNode::Publishers::init(MinimalPublisherNode* node)
{
    publisher_ = node->create_publisher<std_msgs::msg::String>(node->parameters_.topics.publisher_topic, 10);
}

void MinimalPublisherNode::Diagnostics::init(MinimalPublisherNode* node)
{
    updater = std::make_shared<diagnostic_updater::Updater>(node);
    updater->setHardwareID("none");
    updater->add("Node Status", node, &MinimalPublisherNode::checkNodeStatus);

    double freq = 1000.0 / node->parameters_.timer_period;
    min_freq = freq * 0.5;
    max_freq = freq * 2.0;

    topic_diag = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
        node->parameters_.topics.publisher_topic, *updater,
        diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 5));
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
    diagnostics_.topic_diag->tick();
}

void MinimalPublisherNode::subscriptionCallback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    message_ = msg->data;
    // Try running `ros2 topic pub --once /input_topic std_msgs/msg/String "{data: 'Hello from command line'}"` from the command line
}

void MinimalPublisherNode::checkNodeStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (count_ > 0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Node is publishing");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No messages published yet");
    }
    stat.add("Messages published", count_);
    stat.add("Current message", message_);
}

MinimalPublisherNode::~MinimalPublisherNode() 
{
    RCLCPP_INFO(this->get_logger(), "MinimalPublisherNode destroyed");
}

NS_FOOT
