// Copyright 2018, Bosch Software Innovations GmbH.
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

#ifndef ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_
#define ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/subscription.hpp"
#include "rosbag2_transport/logging.hpp"

namespace rosbag2_transport
{

/**
 * This class is an implementation of an rclcpp::Subscription for serialized messages whose topic
 * is not known at compile time (hence templating does not work).
 *
 * It does not support intra-process handling
 */
class GenericSubscription : public rclcpp::SubscriptionBase
{
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(GenericSubscription)

    using Ptr = std::shared_ptr<GenericSubscription>;

    /**
   * Constructor. In order to properly subscribe to a topic, this subscription needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \param node_base NodeBaseInterface pointer used in parts of the setup.
   * \param ts Type support handle
   * \param topic_name Topic name
   * \param transient  if true, subscribe with transient_local
   * \param callback Callback for new messages of serialized form
   */
    GenericSubscription(
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      const rosidl_message_type_support_t & ts,
      const std::string & topic_name,
      const rclcpp::QoS& qos,
      std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback);

    // Same as create_serialized_message() as the subscription is to serialized_messages only
    std::shared_ptr<void> create_message() override;

    std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

    void handle_message( std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override;

    void handle_loaned_message(
      void * loaned_message, const rclcpp::MessageInfo & message_info) override;

    // Same as return_serialized_message() as the subscription is to serialized_messages only
    void return_message(std::shared_ptr<void> & message) override;

    void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override;

  private:
    RCLCPP_DISABLE_COPY(GenericSubscription)

    std::shared_ptr<rclcpp::SerializedMessage> borrow_serialized_message(size_t capacity);
    rcutils_allocator_t default_allocator_;
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback_;
};

//---------- implementation -----------

inline rcl_subscription_options_t PermissiveOptions()
{
  rcl_subscription_options_t options = rcl_subscription_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  options.qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  options.qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  return options;
}

inline rcl_subscription_options_t LatchingOptions()
{
  rcl_subscription_options_t options = rcl_subscription_get_default_options();
  options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  options.qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  options.qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  return options;
}

inline rcl_subscription_options_t GetSubscriptionOptions(const rclcpp::QoS& qos)
{
  rcl_subscription_options_t options = rcl_subscription_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}

inline
GenericSubscription::GenericSubscription(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  const rosidl_message_type_support_t & ts,
  const std::string & topic_name,
  const rclcpp::QoS& qos,
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
: SubscriptionBase(
    node_base,
    ts,
    topic_name,
    GetSubscriptionOptions( qos ),
    true),
  default_allocator_(rcutils_get_default_allocator()),
  callback_(callback)
{}

inline std::shared_ptr<void> GenericSubscription::create_message()
{
    return create_serialized_message();
}

inline std::shared_ptr<rclcpp::SerializedMessage> GenericSubscription::create_serialized_message()
{
    return borrow_serialized_message(0);
}

inline void GenericSubscription::handle_message(
  std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info)
{
    (void) message_info;
    auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
    callback_(typed_message);
}

inline void GenericSubscription::return_message(std::shared_ptr<void> & message)
{
    auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
    return_serialized_message(typed_message);
}

inline void GenericSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
    message.reset();
}

inline std::shared_ptr<rclcpp::SerializedMessage>
GenericSubscription::borrow_serialized_message(size_t capacity)
{
    return std::make_shared<rclcpp::SerializedMessage>(capacity);
}

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__GENERIC_SUBSCRIPTION_HPP_
