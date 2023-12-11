#include "ros2_parser.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>


bool TypeHasHeader(const rosidl_message_type_support_t* type_support)
{
  auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);

  if (members->member_count_ >= 1 && members->members_)
  {
    const rosidl_typesupport_introspection_cpp::MessageMember& first_field = members->members_[0];
    if (first_field.members_ == nullptr)
    {
      return false;
    }
    const auto* header_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(first_field.members_->data);
    if (strcmp(header_members->message_name_, "Header") == 0 && strcmp(header_members->message_namespace_, "std_msgs::"
                                                                                                           "msg") == 0)
    {
      return true;
    }
  }
  return false;
}


Ros2MessageParser::Ros2MessageParser(std::shared_ptr<MessageParser> parser_impl,
                                     const std::string& topic,
                                     const std::string& type,
                                     PlotDataMapRef& plot_data) : 
RosMessageParser(plot_data), _parser_impl(parser_impl)
{
  _info.name = topic;
  _info.type = type;

  _info.introspection_library = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
  _info.introspection_support = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_introspection_cpp", _info.introspection_library);

  auto identifier   = rosidl_typesupport_cpp::typesupport_identifier;
  _info.support_library = rosbag2_cpp::get_typesupport_library(type, identifier);
  _info.type_support = rosbag2_cpp::get_typesupport_handle(type, identifier, _info.support_library);

  _info.has_header_stamp = TypeHasHeader(_info.introspection_support);
}

bool Ros2MessageParser::parseMessage(MessageRef serialized_msg, double& timestamp)
{
  return _parser_impl->parseMessage(serialized_msg, timestamp);
}

//-------------------------------------------------------------------------------------------

void Ros2CompositeParser::registerMessageType(
    const std::string &topic_name,
    const std::string &topic_type)
{
  auto rosx_parser = parserFactories().at("ros2msg")->createParser(topic_name, topic_type, schema, _plot_data); 
  auto parser = std::make_shared<Ros2MessageParser>(rosx_parser, topic_name, topic_type, _plot_data);
  _parsers.insert({topic_name, parser});
}

