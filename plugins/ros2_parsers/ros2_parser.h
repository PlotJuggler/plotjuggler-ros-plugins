#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "ros2_introspection/ros2_introspection.hpp"

#include <PlotJuggler/plotdata.h>
#include "parser_configuration.h"

//namespace ros2
//{
//using MessageRef = rcutils_uint8_array_t;
//}

using namespace PJ;

struct TopicInfo{
  std::string name;
  std::string type;
  const rosidl_message_type_support_t* type_support;
};

class Ros2MessageParser: public RosMessageParser
{
  public:
  Ros2MessageParser(const std::string& topic_name, PJ::PlotDataMapRef& plot_data):
      RosMessageParser(topic_name, plot_data)
  {}

  const rosidl_message_type_support_t* typeSupport() const
  {
    return _type_support;
  }

  protected:
  const rosidl_message_type_support_t* _type_support = nullptr;
};


template <typename T>
class BuiltinMessageParser : public Ros2MessageParser
{
  public:
  BuiltinMessageParser(const std::string& topic_name, PlotDataMapRef& plot_data)
      : Ros2MessageParser(topic_name, plot_data)
  {
    _type_support = rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  }

  virtual bool parseMessage(MessageRef serialized_msg,
                            double& timestamp)
  {
    rcutils_uint8_array_t msg_ref;
    msg_ref.buffer = serialized_msg.data();
    msg_ref.buffer_length = serialized_msg.size();
    T msg;
    if (RMW_RET_OK != rmw_deserialize(&msg_ref, _type_support, &msg))
    {
      throw std::runtime_error("failed to deserialize message");
    }
    parseMessageImpl(msg, timestamp);
    return true;
  }

  virtual void parseMessageImpl(const T& msg, double& timestamp) = 0;
};

class IntrospectionParser : public Ros2MessageParser
{
  public:
  IntrospectionParser(const std::string& topic_name,
                      const std::string& topic_type,
                      PlotDataMapRef& plot_data)
      : Ros2MessageParser(topic_name, plot_data),
      _intropection_parser(topic_name, topic_type)
  {
    _type_support = _intropection_parser.topicInfo().type_support;
  }

  bool parseMessage(MessageRef serialized_msg, double& timestamp) override;

  private:
  Ros2Introspection::Parser _intropection_parser;
  Ros2Introspection::FlatMessage _flat_msg;
};

class Ros2CompositeParser: public CompositeParser
{
  public:

  Ros2CompositeParser(PlotDataMapRef& plot_data): CompositeParser(plot_data) {}

  void registerMessageType(const std::string& topic_name,
                           const std::string& topic_type);

  const rosidl_message_type_support_t* typeSupport(const std::string& topic_name) const;
};


