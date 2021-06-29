cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
project(ros2_mscl_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
"msg/FilterHeading.msg"
"msg/FilterHeadingState.msg"
"msg/FilterStatus.msg"
"msg/GNSSAidingStatus.msg"
"msg/GPSCorrelationTimestamp.msg"
"msg/GPSCorrelationTimestampStamped.msg"
"msg/RTKStatus.msg"
"msg/Status.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
