#include "ros2_parser.h"

#include <set>

#include <rosidl_typesupport_introspection_c/message_introspection.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <fmt/core.h>


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

std::string CreateSchema(const std::string& base_type)
{
  std::string schema;
  using TypeSupport = rosidl_message_type_support_t;
  using namespace rosidl_typesupport_introspection_cpp;

  std::set<std::string> secondary_types_pending;
  std::set<std::string> secondary_types_done;

  auto addTypeToSchema = [&](const std::string& type_name, bool add_header)
  {
    auto introspection_library = rosbag2_cpp::get_typesupport_library(type_name, "rosidl_typesupport_introspection_cpp");
    auto introspection_support = rosbag2_cpp::get_typesupport_handle(type_name, "rosidl_typesupport_introspection_cpp", 
                                                                     introspection_library);

    if(add_header)
    {
      schema += fmt::format("=====================================\nMSG: {}\n", type_name); 
    }
    const auto* members = static_cast<const MessageMembers*>(introspection_support->data);
    for (size_t i = 0; i < members->member_count_; i++)
    {
      const MessageMember& member = members->members_[i];

      switch(member.type_id_)
      {
        case ROS_TYPE_FLOAT32: schema += "float32"; break;
        case ROS_TYPE_FLOAT64: schema += "float64"; break;
        case ROS_TYPE_UINT8:
        case ROS_TYPE_BYTE:
        case ROS_TYPE_CHAR: schema += "uint8"; break;
        case ROS_TYPE_BOOLEAN: schema += "bool"; break;
        case ROS_TYPE_INT8: schema += "int8"; break;
        case ROS_TYPE_UINT16: schema += "uint16"; break;
        case ROS_TYPE_INT16: schema += "int16"; break;
        case ROS_TYPE_UINT32: schema += "uint32"; break;
        case ROS_TYPE_INT32: schema += "int32"; break;
        case ROS_TYPE_UINT64: schema += "uint64"; break;
        case ROS_TYPE_INT64: schema += "int64"; break;
        case ROS_TYPE_STRING: schema += "string"; break;
        case ROS_TYPE_MESSAGE: {
            auto type_info = reinterpret_cast<const MessageMembers*>(member.members_->data);
            std::string package = type_info->message_namespace_;
            package = package.substr(0, package.size() - 5); // remove "::msg"
            const std::string field_type = fmt::format("{}/{}", package, type_info->message_name_);
            schema += field_type;
            if(secondary_types_done.count(field_type) == 0)
            {
              secondary_types_pending.insert(field_type);
            }
          } break;
      }

      if (member.is_array_)
      {
        if(member.array_size_ > 0) {
          schema += fmt::format("[{}]", member.array_size_);
        }
        else {
          schema +="[]";
        }
      }
      schema += fmt::format(" {}\n", member.name_);
    }
  };

  addTypeToSchema(base_type, false);

  while(!secondary_types_pending.empty())
  {
    auto it = secondary_types_pending.begin();
    std::string other_type = *it;
    secondary_types_done.insert(other_type);
    addTypeToSchema(other_type, true);
    secondary_types_pending.erase(it);
  }
  return schema;
}


TopicInfo CreateTopicInfo(const std::string& topic_name, const std::string& type_name)
{
  TopicInfo info;
  info.topic_name = topic_name;
  info.type = type_name;

  info.introspection_library = rosbag2_cpp::get_typesupport_library(type_name, "rosidl_typesupport_introspection_cpp");
  info.introspection_support = rosbag2_cpp::get_typesupport_handle(type_name, "rosidl_typesupport_introspection_cpp", info.introspection_library);

  auto identifier   = rosidl_typesupport_cpp::typesupport_identifier;
  info.support_library = rosbag2_cpp::get_typesupport_library(type_name, identifier);
  info.type_support = rosbag2_cpp::get_typesupport_handle(type_name, identifier, info.support_library);

  info.has_header_stamp = TypeHasHeader(info.introspection_support);
  return info;
}

std::shared_ptr<PJ::MessageParser> CreateParserROS2(const PJ::ParserFactories& factories,
                                                    const std::string& topic_name,
                                                    const std::string& type_name,
                                                    PJ::PlotDataMapRef& data)
{
  return factories.at("ros2msg")->createParser(topic_name, type_name, CreateSchema(type_name), data);
}