/*
 * Wrappers to keep compatibility with all active ROS versions as of 01/2026 (Kilted, Jazzy, Humble). Dropping of
 * this file should be reevaluated once Humble goes end of life (07/2027).
 */

#ifndef TYPESUPPORT_WRAPPER_H
#define TYPESUPPORT_WRAPPER_H

#include <memory>
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

namespace wrapper
{
std::shared_ptr<rcpputils::SharedLibrary> get_typesupport_library(const std::string& type,
                                                                  const std::string& typesupport_identifier);

const rosidl_message_type_support_t* get_message_typesupport_handle(const std::string& type,
                                                                    const std::string& typesupport_identifier,
                                                                    std::shared_ptr<rcpputils::SharedLibrary> library);
}  // namespace wrapper

#endif  // TYPESUPPORT_WRAPPER_H
