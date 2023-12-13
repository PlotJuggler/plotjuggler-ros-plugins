# Qt related stuff
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

QT5_WRAP_UI ( COMMON_UI_SRC
    qnodedialog.ui
    rule_editing.ui
    dialog_select_ros_topics.ui
    dialog_with_itemlist.ui
    publisher_select_dialog.ui
    )

SET( COMMON_SRC
    dialog_select_ros_topics.h
    dialog_select_ros_topics.cpp
    dialog_with_itemlist.h
    publisher_select_dialog.h
    parser_configuration.cpp
    parser_configuration.h
   )

#############

if(COMPILING_WITH_CATKIN)

    include_directories(
        ${catkin_INCLUDE_DIRS}  )

     list(APPEND COMMON_SRC
        qnodedialog.cpp
        rule_editing.cpp
        )

    add_library( commonROS STATIC ${COMMON_SRC} ${COMMON_UI_SRC} )
    target_compile_definitions(commonROS PUBLIC DISABLE_RULE_EDITING)
    target_link_libraries( commonROS PRIVATE
        ${Qt5Widgets_LIBRARIES}
        ${Qt5Xml_LIBRARIES}
        ${catkin_LIBRARIES})

    #########
    add_library( DataLoadROS SHARED
        DataLoadROS/dataload_ros.cpp)
    target_link_libraries( DataLoadROS  commonROS)
    #############
    # add_library( DataStreamROS SHARED
    #     DataStreamROS/datastream_ROS.cpp )
    # target_link_libraries( DataStreamROS  commonROS )
    # #############
    # add_library( RosTopicPublisher SHARED
    #     TopicPublisherROS/statepublisher_rostopic.cpp)
    # target_link_libraries( RosTopicPublisher commonROS )
    # #############
    # QT5_WRAP_UI ( ROSOUT_UI RosoutPublisher/logwidget.ui )
    # add_library( RosoutPublisher SHARED ${ROSOUT_UI}
    #     RosoutPublisher/rosout_publisher.cpp
    #     RosoutPublisher/logs_table_model.cpp
    #     RosoutPublisher/logwidget.cpp
    #     RosoutPublisher/modelfilter.cpp
    #     )
    # target_link_libraries( RosoutPublisher commonROS )
#######################################################################
elseif(COMPILING_WITH_AMENT)

    include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

    list( APPEND COMMON_SRC ros_parsers/ros2_parser.cpp )

    set(AMENT_DEPENDENCIES
        rclcpp
        rosbag2
        rcpputils
        rosbag2_transport
        tf2_msgs
        tf2_ros
        plotjuggler
        )

    add_library( commonROS STATIC ${COMMON_SRC} ${COMMON_UI_SRC})

    target_compile_definitions(commonROS PUBLIC DISABLE_RULE_EDITING)
    target_link_libraries( commonROS
        ${Qt5Widgets_LIBRARIES}
        ${Qt5Xml_LIBRARIES})

    ament_target_dependencies( commonROS ${AMENT_DEPENDENCIES})

    #############
    add_library( DataLoadROS2 SHARED
        DataLoadROS2/dataload_ros2.cpp)
    target_link_libraries( DataLoadROS2 commonROS)
    ament_target_dependencies( DataLoadROS2 ${AMENT_DEPENDENCIES})
   #############
    add_library( DataStreamROS2 SHARED
        DataStreamROS2/datastream_ros2.cpp)
    target_link_libraries( DataStreamROS2  commonROS)
    ament_target_dependencies( DataStreamROS2 ${AMENT_DEPENDENCIES})
   #############
    add_library( TopicPublisherROS2 SHARED
        TopicPublisherROS2/publisher_ros2.cpp)
    target_link_libraries( TopicPublisherROS2 commonROS)
    ament_target_dependencies( TopicPublisherROS2 ${AMENT_DEPENDENCIES})

    if("$ENV{AMENT_PREFIX_PATH}" EQUAL "/opt/ros/humble")
        target_compile_definitions(TopicPublisherROS2 PUBLIC HUMBLE)
    endif()

    add_executable(test_ros2_parser
        ros_parsers/test_ros2_parser.cpp 
        ros_parsers/ros2_parser.cpp)
    target_link_libraries( test_ros2_parser commonROS)
    ament_target_dependencies( test_ros2_parser ${AMENT_DEPENDENCIES})

endif()

#######################################################################

if(COMPILING_WITH_CATKIN)
    install(TARGETS
        DataLoadROS
        # DataStreamROS
        # RosTopicPublisher
        # RosoutPublisher
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/
        )
elseif(COMPILING_WITH_AMENT)
    install(TARGETS
        test_ros2_parser
        DataLoadROS2
        DataStreamROS2
        TopicPublisherROS2
        DESTINATION  lib/${PROJECT_NAME}  )
endif()
