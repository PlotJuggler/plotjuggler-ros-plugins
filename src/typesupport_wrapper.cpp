

#include "typesupport_wrapper.h"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

#ifdef ROS_HUMBLE
#include "rosbag2_cpp/typesupport_helpers.hpp"
#else
#include "rclcpp/typesupport_helpers.hpp"
#endif

namespace wrapper
{
std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(const std::string& type,
                                                                  const std::string& typesupport_identifier)
{
#ifdef ROS_HUMBLE
  return rosbag2_cpp::get_typesupport_library(type, typesupport_identifier);
#else
  return rclcpp::get_typesupport_library(type, typesupport_identifier);
#endif
}

const rosidl_message_type_support_t* get_message_typesupport_handle(const std::string& type,
                                                                    const std::string& typesupport_identifier,
                                                                    std::shared_ptr<rcpputils::SharedLibrary> library)
{
#ifdef ROS_HUMBLE
  return rosbag2_cpp::get_typesupport_handle(type, typesupport_identifier, library);
#else
  return rclcpp::get_message_typesupport_handle(type, typesupport_identifier, *library);
#endif
}
}  // namespace wrapper
