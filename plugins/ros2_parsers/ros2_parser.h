#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <PlotJuggler/plotdata.h>
#include "parser_configuration.h"


using namespace PJ;


struct TopicInfo{

  std::string name;
  std::string type;
  bool has_header_stamp;

  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t *introspection_support;

  std::shared_ptr<rcpputils::SharedLibrary> support_library;
  const rosidl_message_type_support_t *type_support;

  static rcutils_allocator_t allocator;
};

class Ros2MessageParser: public RosMessageParser
{
public:
  Ros2MessageParser(std::shared_ptr<MessageParser> parser_impl,
                    const std::string& topic_name,
                    const std::string& topic_type,
                    PlotDataMapRef& plot_data);
  
  bool parseMessage(MessageRef serialized_msg, double& timestamp) override;

protected:
  TopicInfo _info;
  std::shared_ptr<MessageParser> _parser_impl;
 };


class Ros2CompositeParser: public CompositeParser
{
  public:

  Ros2CompositeParser(PlotDataMapRef& plot_data): CompositeParser(plot_data) {}

  void registerMessageType(const std::string& topic_name,
                           const std::string& topic_type);
};


