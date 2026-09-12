#pragma once  // Favor using this over the #ifndef, #define method

// First include your local package stuff
#include "package_defs.hpp"  //  This is where we include all our namespace stuff for the package

// then include external libary stuff
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

NS_HEAD  // macro for consistantly defining our namespace for the package

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
        int timer_period = 500; ///< Timer period in milliseconds

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

            void declare(MinimalPublisherNode* node, const std::string& prefix);
            void update(MinimalPublisherNode* node, const std::string& prefix);
            rclcpp::QoS makeQos() const;
        };

        struct Topics{
            std::string subscriber_topic = "input_topic";  ///< Name of the input topic
            std::string publisher_topic = "output_topic";  ///< Name of the output topic
        } topics;

        QosSettings subscriber_qos; ///< QoS used by the input subscription
        QosSettings publisher_qos;  ///< QoS used by the output publisher

        Parameters();
        void declare(MinimalPublisherNode* node);
        void update(MinimalPublisherNode* node);
    };

    /**
     * @brief Structure for holding subscriptions for the MinimalPublisherNode class.
     */
    struct Subscribers
    {
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; ///< Shared pointer to the subscription
        void init(MinimalPublisherNode* node);
    };

    /**
     * @brief Structure for holding publishers for the MinimalPublisherNode class.
     */
    struct Publishers
    {
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; ///< Shared pointer to the publisher
        void init(MinimalPublisherNode* node);
    };

    /**
     * @brief Structure for holding diagnostics components for the MinimalPublisherNode class.
     *
     * Uses diagnostic_updater to periodically publish diagnostic status and
     * monitor the publication frequency of the output topic.
     */
    struct Diagnostics
    {
        std::shared_ptr<diagnostic_updater::Updater> updater; ///< Shared pointer to the diagnostic updater
        void init(MinimalPublisherNode* node);
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
    void subscriptionCallback(std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief Diagnostic callback for reporting node status.
     *
     * This function is registered with the diagnostic updater and called
     * periodically to report the health and state of the node.
     *
     * @param stat Wrapper for building the diagnostic status message.
     */
    void checkNodeStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

    Parameters parameters_;
    Subscribers subscribers_;
    Publishers publishers_;
    Diagnostics diagnostics_;

    std::string message_;
    rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
    size_t count_; ///< Counter for the number of messages published
};

NS_FOOT
