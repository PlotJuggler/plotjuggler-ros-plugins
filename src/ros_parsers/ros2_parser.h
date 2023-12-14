#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/types.h"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <PlotJuggler/plotdata.h>
#include "parser_configuration.h"

struct TopicInfo
{

  std::string topic_name;
  std::string type;
  bool has_header_stamp;

  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t *introspection_support;

  std::shared_ptr<rcpputils::SharedLibrary> support_library;
  const rosidl_message_type_support_t *type_support;

  static rcutils_allocator_t allocator;
};

std::string CreateSchema(const std::string &type_name);

TopicInfo CreateTopicInfo(const std::string &topic_name, const std::string &type_name);

std::shared_ptr<PJ::MessageParser>
CreateParserROS2(const PJ::ParserFactories &factories,
                 const std::string &topic_name,
                 const std::string &type_name,
                 PJ::PlotDataMapRef &data);
