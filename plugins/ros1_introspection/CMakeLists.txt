cmake_minimum_required(VERSION 2.8.3)
project(ros_type_introspection)

find_package(Boost REQUIRED COMPONENTS regex)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(ros_type_introspection_static STATIC
   src/ros_type.cpp
   src/ros_field.cpp
   src/stringtree_leaf.cpp
   src/ros_message.cpp
   src/substitution_rule.cpp
   src/ros_introspection.cpp
 )

target_link_libraries(ros_type_introspection_static  ${catkin_LIBRARIES})


