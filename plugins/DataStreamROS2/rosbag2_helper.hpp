#ifndef ROSBAG2_HELPER_HPP
#define ROSBAG2_HELPER_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace PJ
{

inline rclcpp::QoS adapt_request_to_offers(
    const std::string & topic_name, const std::vector<rclcpp::TopicEndpointInfo> & endpoints)
{
  rclcpp::QoS request_qos = rclcpp::SystemDefaultsQoS();
  request_qos.best_effort().durability_volatile();

  if (endpoints.empty()) {
    return request_qos;
  }
  size_t num_endpoints = endpoints.size();
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto & endpoint : endpoints) {
    const auto & profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
  }

  // We set policies in order as defined in rmw_qos_profile_t
  // Policy: history, depth
  // History does not affect compatibility

  // Policy: reliability
  if (reliability_reliable_endpoints_count == num_endpoints) {
    request_qos.reliable();
  } else {
    if (reliability_reliable_endpoints_count > 0) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM( topic_name <<
          ": some, but not all, publishers on topic "
          "are offering RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
          "Falling back to RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT "
          "as it will connect to all publishers. "
          "Some messages from Reliable publishers could be dropped.");
    }
    request_qos.best_effort();
  }

  // Policy: durability
  // If all publishers offer transient_local, we can request it and receive latched messages
  if (durability_transient_local_endpoints_count == num_endpoints) {
    request_qos.transient_local();
  } else {
    if (durability_transient_local_endpoints_count > 0) {
      ROSBAG2_TRANSPORT_LOG_WARN_STREAM(topic_name <<
          ": some, but not all, publishers on topic "
          "are offering RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
          "Falling back to RMW_QOS_POLICY_DURABILITY_VOLATILE "
          "as it will connect to all publishers. "
          "Previously-published latched messages will not be retrieved.");
    }
    request_qos.durability_volatile();
  }
  // Policy: deadline
  // Deadline does not affect delivery of messages,
  // and we do not record Deadline"Missed events.
  // We can always use unspecified deadline, which will be compatible with all publishers.

  // Policy: lifespan
  // Lifespan does not affect compatibiliy

  // Policy: liveliness, liveliness_lease_duration
  // Liveliness does not affect delivery of messages,
  // and we do not record LivelinessChanged events.
  // We can always use unspecified liveliness, which will be compatible with all publishers.
  return request_qos;
}

}

#endif // ROSBAG2_HELPER_HPP
