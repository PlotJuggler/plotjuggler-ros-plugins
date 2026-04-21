#include "typesupport_wrapper.h"

#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

namespace wrapper
{
std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(const std::string& type,
                                                                  const std::string& typesupport_identifier)
{
  return rclcpp::get_typesupport_library(type, typesupport_identifier);
}

const rosidl_message_type_support_t* get_message_typesupport_handle(const std::string& type,
                                                                    const std::string& typesupport_identifier,
                                                                    std::shared_ptr<rcpputils::SharedLibrary> library)
{
  // rclcpp::get_message_typesupport_handle was introduced in Jazzy (rclcpp 28.x).
  // On Humble/Iron the only name available is rclcpp::get_typesupport_handle.
#if RCLCPP_VERSION_GTE(28, 0, 0)
  return rclcpp::get_message_typesupport_handle(type, typesupport_identifier, *library);
#else
  return rclcpp::get_typesupport_handle(type, typesupport_identifier, *library);
#endif
}
}  // namespace wrapper
