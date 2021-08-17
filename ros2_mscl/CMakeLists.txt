cmake_minimum_required(VERSION 3.5)
project(ros2_mscl)

# Locate the common code and messages
set(COMMON_NAME "ros_mscl_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/../${COMMON_NAME}")

# Download and install MSCL
include(${COMMON_DIR}/cmake/download_mscl.cmake)
download_mscl(VERSION "62.0.0")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-format -fno-builtin-memcpy)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mscl_msgs REQUIRED)

# Let the code know if it is being compiled with ROS1 or ROS2
if(DEFINED ENV{ROS_VERSION})
  add_definitions(-DMICROSTRAIN_ROS_VERSION=$ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not set.")
endif()

ament_export_include_directories()
ament_export_dependencies(    
    roscpp
    rosidl_default_runtime
    cmake_modules
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
		mscl_msgs
)
ament_package()

###########
## Build ##
###########


include_directories(${CMAKE_SOURCE_DIR}/include ${COMMON_DIR}/include ${MSCL_INC_PATH} ${BOOST_INC_PATH})

# Executables/Linking
add_executable(ros2_mscl_node
  ${COMMON_DIR}/src/microstrain_subscribers.cpp
  ${COMMON_DIR}/src/microstrain_publishers.cpp
  ${COMMON_DIR}/src/microstrain_node_base.cpp
  ${COMMON_DIR}/src/microstrain_services.cpp
  ${COMMON_DIR}/src/microstrain_parser.cpp
  ${COMMON_DIR}/src/microstrain_config.cpp
  src/microstrain_3dm.cpp
  src/microstrain_3dm_node.cpp
)
ament_target_dependencies(ros2_mscl_node rclcpp rclcpp_lifecycle std_msgs std_srvs lifecycle_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs mscl_msgs) 
target_link_libraries(ros2_mscl_node ${MSCL_LIB_PATH})

#############
## Install ##
#############


install(TARGETS
  ros2_mscl_node
  DESTINATION share/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()
