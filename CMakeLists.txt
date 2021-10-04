cmake_minimum_required(VERSION 3.5)

PROJECT(plotjuggler_ros)

# http://answers.ros.org/question/230877/optionally-build-a-package-with-catkin/
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(COMPILING_WITH_CATKIN 1)

    message(STATUS "---------------------------------------------------------------------")
    message(STATUS "PlotJuggler is being built using CATKIN. ROS plugins will be compiled")
    message(STATUS "---------------------------------------------------------------------")

    set(ROS_DEPENDENCIES
        rosbag_storage
        roscpp
        roscpp_serialization
        tf
        tf2_ros
        diagnostic_msgs
        nav_msgs
        sensor_msgs
        plotjuggler_msgs
       #  pal_statistics_msgs
        plotjuggler )

    find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )

    catkin_package(
        CATKIN_DEPENDS ${ROS_DEPENDENCIES}  )

elseif( DEFINED ENV{AMENT_PREFIX_PATH})
    set(COMPILING_WITH_AMENT 1)
    
    message(STATUS "---------------------------------------------------------------------")
    message(STATUS "PlotJuggler is being built using AMENT. ROS2 plugins will be compiled")
    message(STATUS "---------------------------------------------------------------------")

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rcpputils REQUIRED)
    find_package(rosbag2 REQUIRED)
    find_package(rosbag2_transport REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    find_package(fastcdr REQUIRED)
    find_package(Boost REQUIRED)
    find_package(plotjuggler_msgs REQUIRED)
    find_package(tf2_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(plotjuggler REQUIRED)
   #  find_package(pal_statistics_msgs REQUIRED)

else()
    message(FATAL_ERROR "PlotJuggler is being WITHOUT any ROS support")
endif()

cmake_policy (SET CMP0020 NEW)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fno-omit-frame-pointer")
endif()

if(APPLE AND EXISTS /usr/local/opt/qt5)
    # Homebrew installs Qt5 (up to at least 5.9.1) in
    # /usr/local/qt5, ensure it can be found by CMake since
    # it is not in the default /usr/local prefix.
    # source: https://github.com/Homebrew/homebrew-core/issues/8392#issuecomment-325226494
    list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt5")
    set(CMAKE_MACOSX_RPATH 1)
endif()


find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    Concurrent
    Xml
    Svg
    WebSockets)

set( QT_LINK_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Xml
    Qt5::Svg
    Qt5::WebSockets
    )

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

IF (NOT WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC")
ENDIF()

include_directories(
    3rdparty
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5WebSockets_INCLUDE_DIRS}
    ${Qt5Svg_INCLUDE_DIRS}
)

add_definitions(-DFMT_HEADER_ONLY)

#########################  INSTALL ####################################

if(COMPILING_WITH_CATKIN)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

	install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME})

elseif(COMPILING_WITH_AMENT)
    find_package(ament_cmake REQUIRED)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

	install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME})
    
else()
    set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/plotjuggler)
endif()

#############################################################

add_subdirectory( plugins )

if(COMPILING_WITH_AMENT)
    ament_package()
endif()

