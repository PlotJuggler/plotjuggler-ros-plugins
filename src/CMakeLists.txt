add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( COMMON_UI_SRC
    qnodedialog.ui
    dialog_select_ros_topics.ui
    dialog_with_itemlist.ui
    publisher_select_dialog.ui )


add_library( commonROS STATIC 
   dialog_select_ros_topics.h
    dialog_select_ros_topics.cpp
    dialog_with_itemlist.h
    publisher_select_dialog.h
    parser_configuration.cpp
    parser_configuration.h
    ros_parsers/ros2_parser.cpp
    ${COMMON_UI_SRC}
)

target_link_libraries( commonROS
    PUBLIC
        Qt5::Widgets
        Qt5::Xml
        rclcpp::rclcpp
        rcpputils::rcpputils
        rosbag2_transport::rosbag2_transport
        tf2_ros::tf2_ros
        plotjuggler::plotjuggler_base
        ${tf2_msgs_TARGETS}
    )

#######################################################################
add_library( DataLoadROS2 SHARED    DataLoadROS2/dataload_ros2.cpp)
target_link_libraries( DataLoadROS2 commonROS)


add_library( DataStreamROS2 SHARED  DataStreamROS2/datastream_ros2.cpp)
target_link_libraries( DataStreamROS2  commonROS)


add_library( TopicPublisherROS2 SHARED TopicPublisherROS2/publisher_ros2.cpp)
target_link_libraries( TopicPublisherROS2 commonROS)

message("AMENT_PREFIX_PATH = ${AMENT_PREFIX_PATH}")
message("ROS_DISTRO (env) = $ENV{ROS_DISTRO}")

if("${AMENT_PREFIX_PATH}" STREQUAL "/opt/ros/humble" OR "$ENV{ROS_DISTRO}" STREQUAL "humble")
        message(STATUS "Detected Humble")
        target_compile_definitions(DataLoadROS2 PUBLIC ROS_HUMBLE)
        target_compile_definitions(TopicPublisherROS2 PUBLIC ROS_HUMBLE)
endif()

#######################################################################


install(TARGETS
        DataLoadROS2
        DataStreamROS2
        TopicPublisherROS2
        DESTINATION  lib/${PROJECT_NAME}  
    )

