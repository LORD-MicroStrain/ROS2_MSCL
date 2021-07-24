cmake_minimum_required(VERSION 3.5)
project(ros2_mscl)

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
#find_package(roslint REQUIRED)
#find_package(message_generation REQUIRED)
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
#find_package(diagnostic_updater REQUIRED)
find_package(ros2_mscl_msgs REQUIRED)


find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "/usr/share/c++-mscl" DOC "MSCL Library" NO_DEFAULT_PATH)
set(MSCL_INC_PATH "/usr/share/c++-mscl/source")
set(MSCL_LIB "/usr/share/c++-mscl")

set(srv_files
  "srv/DeviceSettings.srv"
  "srv/ExternalHeadingUpdate.srv"
  "srv/GetAccelAdaptiveVals.srv"
  "srv/GetAccelBias.srv"
  "srv/GetAccelBiasModel.srv"
  "srv/GetAccelNoise.srv"
  "srv/GetComplementaryFilter.srv"
  "srv/GetConingScullingComp.srv"
  "srv/GetDynamicsMode.srv"
  "srv/GetEstimationControlFlags.srv"
  "srv/GetGravityAdaptiveVals.srv"
  "srv/GetGyroBias.srv"
  "srv/GetGyroBiasModel.srv"
  "srv/GetGyroNoise.srv"
  "srv/GetHardIronValues.srv"
  "srv/GetHeadingSource.srv"
  "srv/GetMagAdaptiveVals.srv"
  "srv/GetMagDipAdaptiveVals.srv"
  "srv/GetMagNoise.srv"
  "srv/GetReferencePosition.srv"
  "srv/GetRelativePositionReference.srv"
  "srv/GetSensor2VehicleOffset.srv"
  "srv/GetSensor2VehicleRotation.srv"
  "srv/GetSensor2VehicleTransformation.srv"
  "srv/GetSoftIronMatrix.srv"
  "srv/GetTareOrientation.srv"
  "srv/GetZeroAngleUpdateThreshold.srv"
  "srv/GetZeroVelocityUpdateThreshold.srv"
  "srv/InitFilterEuler.srv"
  "srv/InitFilterHeading.srv"
  "srv/SetAccelAdaptiveVals.srv"
  "srv/SetAccelBias.srv"
  "srv/SetAccelBiasModel.srv"
  "srv/SetAccelNoise.srv"
  "srv/SetComplementaryFilter.srv"
  "srv/SetConingScullingComp.srv"
  "srv/SetDynamicsMode.srv"
  "srv/SetEstimationControlFlags.srv"
  "srv/SetGravityAdaptiveVals.srv"
  "srv/SetGyroBias.srv"
  "srv/SetGyroBiasModel.srv"
  "srv/SetGyroNoise.srv"
  "srv/SetHardIronValues.srv"
  "srv/SetHeadingSource.srv"
  "srv/SetMagAdaptiveVals.srv"
  "srv/SetMagDipAdaptiveVals.srv"
  "srv/SetMagNoise.srv"
  "srv/SetReferencePosition.srv"
  "srv/SetRelativePositionReference.srv"
  "srv/SetSensor2VehicleOffset.srv"
  "srv/SetSensor2VehicleRotation.srv"
  "srv/SetSoftIronMatrix.srv"
  "srv/SetTareOrientation.srv"
  "srv/SetZeroAngleUpdateThreshold.srv"
  "srv/SetZeroVelocityUpdateThreshold.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files} DEPENDENCIES std_msgs geometry_msgs)


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
		ros2_mscl_msgs
)

ament_package()

###########
## Build ##
###########


include_directories(${CMAKE_SOURCE_DIR}/include/ros2_mscl ${MIPLIBINC} ${MIPUTILINC} ${MSCL_INC_PATH} ${MSCL_LIB} ${MSCL_INC})

# Executables/Linking
add_executable(ros2_mscl_node src/microstrain_3dm.cpp src/microstrain_3dm_node.cpp)
ament_target_dependencies(ros2_mscl_node rclcpp rclcpp_lifecycle std_msgs std_srvs lifecycle_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs ros2_mscl_msgs) 
rosidl_target_interfaces(ros2_mscl_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
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

#############
## Testing ##
#############

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#endif()

## Run roslint on only the node files for now
# TODO: Add more files as needed
# roslint_cpp(src/microstrain_3dm.cpp src/microstrain_3dm_node.cpp)
