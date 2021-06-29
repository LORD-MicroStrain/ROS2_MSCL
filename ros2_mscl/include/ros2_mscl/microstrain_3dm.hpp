/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord ROS2 Inertial Driver Definition File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _MICROSTRAIN_3DM_H
#define _MICROSTRAIN_3DM_H

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//MSCL
#include "mscl/mscl.h"
#include "ros2_mscl_msgs/msg/status.hpp"
#include "ros2_mscl_msgs/msg/rtk_status.hpp"
#include "ros2_mscl_msgs/msg/filter_status.hpp"
#include "ros2_mscl_msgs/msg/filter_heading.hpp"
#include "ros2_mscl_msgs/msg/filter_heading_state.hpp"
#include "ros2_mscl_msgs/msg/gps_correlation_timestamp_stamped.hpp"
#include "ros2_mscl_msgs/msg/gnss_aiding_status.hpp"
#include "ros2_mscl/srv/set_accel_bias.hpp"
#include "ros2_mscl/srv/get_accel_bias.hpp"
#include "ros2_mscl/srv/set_gyro_bias.hpp"
#include "ros2_mscl/srv/get_gyro_bias.hpp"
#include "ros2_mscl/srv/set_hard_iron_values.hpp"
#include "ros2_mscl/srv/get_hard_iron_values.hpp"
#include "ros2_mscl/srv/set_soft_iron_matrix.hpp"
#include "ros2_mscl/srv/get_soft_iron_matrix.hpp"
#include "ros2_mscl/srv/set_complementary_filter.hpp"
#include "ros2_mscl/srv/get_complementary_filter.hpp"
#include "ros2_mscl/srv/init_filter_euler.hpp"
#include "ros2_mscl/srv/init_filter_heading.hpp"
#include "ros2_mscl/srv/device_settings.hpp"
#include "ros2_mscl/srv/set_accel_bias_model.hpp"
#include "ros2_mscl/srv/get_accel_bias_model.hpp"
#include "ros2_mscl/srv/set_gravity_adaptive_vals.hpp"
#include "ros2_mscl/srv/get_gravity_adaptive_vals.hpp"
#include "ros2_mscl/srv/set_sensor2_vehicle_rotation.hpp"
#include "ros2_mscl/srv/get_sensor2_vehicle_rotation.hpp"
#include "ros2_mscl/srv/set_sensor2_vehicle_offset.hpp"
#include "ros2_mscl/srv/get_sensor2_vehicle_offset.hpp"
#include "ros2_mscl/srv/set_reference_position.hpp"
#include "ros2_mscl/srv/get_reference_position.hpp"
#include "ros2_mscl/srv/set_coning_sculling_comp.hpp"
#include "ros2_mscl/srv/get_coning_sculling_comp.hpp"
#include "ros2_mscl/srv/set_estimation_control_flags.hpp"
#include "ros2_mscl/srv/get_estimation_control_flags.hpp"
#include "ros2_mscl/srv/set_dynamics_mode.hpp"
#include "ros2_mscl/srv/get_dynamics_mode.hpp"
#include "ros2_mscl/srv/set_zero_angle_update_threshold.hpp"
#include "ros2_mscl/srv/get_zero_angle_update_threshold.hpp"
#include "ros2_mscl/srv/set_zero_velocity_update_threshold.hpp"
#include "ros2_mscl/srv/get_zero_velocity_update_threshold.hpp"
#include "ros2_mscl/srv/set_tare_orientation.hpp"
#include "ros2_mscl/srv/set_accel_noise.hpp"
#include "ros2_mscl/srv/get_accel_noise.hpp"
#include "ros2_mscl/srv/set_gyro_noise.hpp"
#include "ros2_mscl/srv/get_gyro_noise.hpp"
#include "ros2_mscl/srv/set_mag_noise.hpp"
#include "ros2_mscl/srv/get_mag_noise.hpp"
#include "ros2_mscl/srv/set_gyro_bias_model.hpp"
#include "ros2_mscl/srv/get_gyro_bias_model.hpp"
#include "ros2_mscl/srv/set_mag_adaptive_vals.hpp"
#include "ros2_mscl/srv/get_mag_adaptive_vals.hpp"
#include "ros2_mscl/srv/set_mag_dip_adaptive_vals.hpp"
#include "ros2_mscl/srv/get_mag_dip_adaptive_vals.hpp"
#include "ros2_mscl/srv/set_heading_source.hpp"
#include "ros2_mscl/srv/get_heading_source.hpp"
#include "ros2_mscl/srv/get_sensor2_vehicle_transformation.hpp"
#include "ros2_mscl/srv/external_heading_update.hpp"
#include "ros2_mscl/srv/set_relative_position_reference.hpp"
#include "ros2_mscl/srv/get_relative_position_reference.hpp"


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

#define SECS_PER_WEEK (60L*60*24*7)
#define UTC_GPS_EPOCH_DUR (315964800)

#define USTRAIN_G 9.80665  // from section 5.1.1 in https://www.microstrain.com/sites/default/files/3dm-gx5-25_dcp_manual_8500-0065_reference_document.pdf

//Macro to cause Sleep call to behave as it does for windows
#define Sleep(x) usleep(x*1000.0)

#define GNSS1_ID 0
#define GNSS2_ID 1
#define NUM_GNSS 2


/////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief Contains functions for micostrain driver
///
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain 
{
  ///
  /// \brief Microstrain class
  ///

  class Microstrain : public rclcpp_lifecycle::LifecycleNode
  {
  public:

    Microstrain();
    ~Microstrain() = default;

    bool configure_node();
    bool activate_node();
    bool deactivate_node();
    bool shutdown_or_cleanup_node();


    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &prev_state);

    void parsing_timer_callback();
    void parse_single_mip_packet(const mscl::MipDataPacket& packet);
    void parse_imu_packet(const mscl::MipDataPacket& packet);
    void parse_filter_packet(const mscl::MipDataPacket& packet);
    void parse_gnss_packet(const mscl::MipDataPacket& packet, int gnss_id);
    void parse_rtk_packet(const mscl::MipDataPacket& packet);

    void device_status_callback();
    /*
    void device_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void get_basic_status(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void get_diagnostic_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    */
    void get_accel_bias(const std::shared_ptr<ros2_mscl::srv::GetAccelBias::Request> req, std::shared_ptr<ros2_mscl::srv::GetAccelBias::Response> res);
    void set_accel_bias(const std::shared_ptr<ros2_mscl::srv::SetAccelBias::Request> req, std::shared_ptr<ros2_mscl::srv::SetAccelBias::Response> res);
    
    void get_gyro_bias(const std::shared_ptr<ros2_mscl::srv::GetGyroBias::Request> req, std::shared_ptr<ros2_mscl::srv::GetGyroBias::Response> res);
    void set_gyro_bias(const std::shared_ptr<ros2_mscl::srv::SetGyroBias::Request> req, std::shared_ptr<ros2_mscl::srv::SetGyroBias::Response> res);

    void gyro_bias_capture(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void get_hard_iron_values(const std::shared_ptr<ros2_mscl::srv::GetHardIronValues::Request> req, std::shared_ptr<ros2_mscl::srv::GetHardIronValues::Response> res);
    void set_hard_iron_values(const std::shared_ptr<ros2_mscl::srv::SetHardIronValues::Request> req, std::shared_ptr<ros2_mscl::srv::SetHardIronValues::Response> res);

    void set_soft_iron_matrix(const std::shared_ptr<ros2_mscl::srv::SetSoftIronMatrix::Request> req, std::shared_ptr<ros2_mscl::srv::SetSoftIronMatrix::Response> res);
    void get_soft_iron_matrix(const std::shared_ptr<ros2_mscl::srv::GetSoftIronMatrix::Request> req, std::shared_ptr<ros2_mscl::srv::GetSoftIronMatrix::Response> res);

    void set_complementary_filter(const std::shared_ptr<ros2_mscl::srv::SetComplementaryFilter::Request> req, std::shared_ptr<ros2_mscl::srv::SetComplementaryFilter::Response> res);
    void get_complementary_filter(const std::shared_ptr<ros2_mscl::srv::GetComplementaryFilter::Request> req, std::shared_ptr<ros2_mscl::srv::GetComplementaryFilter::Response> res);

    void set_coning_sculling_comp(const std::shared_ptr<ros2_mscl::srv::SetConingScullingComp::Request> req, std::shared_ptr<ros2_mscl::srv::SetConingScullingComp::Response> res);
    void get_coning_sculling_comp(const std::shared_ptr<ros2_mscl::srv::GetConingScullingComp::Request> req, std::shared_ptr<ros2_mscl::srv::GetConingScullingComp::Response> res);

    void set_sensor2vehicle_rotation(const std::shared_ptr<ros2_mscl::srv::SetSensor2VehicleRotation::Request> req, std::shared_ptr<ros2_mscl::srv::SetSensor2VehicleRotation::Response> res);
    void get_sensor2vehicle_rotation(const std::shared_ptr<ros2_mscl::srv::GetSensor2VehicleRotation::Request> req, std::shared_ptr<ros2_mscl::srv::GetSensor2VehicleRotation::Response> res);

    void set_sensor2vehicle_offset(const std::shared_ptr<ros2_mscl::srv::SetSensor2VehicleOffset::Request> req, std::shared_ptr<ros2_mscl::srv::SetSensor2VehicleOffset::Response> res);
    void get_sensor2vehicle_offset(const std::shared_ptr<ros2_mscl::srv::GetSensor2VehicleOffset::Request> req, std::shared_ptr<ros2_mscl::srv::GetSensor2VehicleOffset::Response> res);

    void get_sensor2vehicle_transformation(const std::shared_ptr<ros2_mscl::srv::GetSensor2VehicleTransformation::Request> req, std::shared_ptr<ros2_mscl::srv::GetSensor2VehicleTransformation::Response> res);
  
    void reset_filter(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  
    void init_filter_heading(const std::shared_ptr<ros2_mscl::srv::InitFilterHeading::Request> req, std::shared_ptr<ros2_mscl::srv::InitFilterHeading::Response> res);
    void init_filter_euler(const std::shared_ptr<ros2_mscl::srv::InitFilterEuler::Request> req, std::shared_ptr<ros2_mscl::srv::InitFilterEuler::Response> res);
  
    void set_heading_source(const std::shared_ptr<ros2_mscl::srv::SetHeadingSource::Request> req, std::shared_ptr<ros2_mscl::srv::SetHeadingSource::Response> res);
    void get_heading_source(const std::shared_ptr<ros2_mscl::srv::GetHeadingSource::Request> req, std::shared_ptr<ros2_mscl::srv::GetHeadingSource::Response> res);

    void set_reference_position(const std::shared_ptr<ros2_mscl::srv::SetReferencePosition::Request> req, std::shared_ptr<ros2_mscl::srv::SetReferencePosition::Response> res);
    void get_reference_position(const std::shared_ptr<ros2_mscl::srv::GetReferencePosition::Request> req, std::shared_ptr<ros2_mscl::srv::GetReferencePosition::Response> res);

    void set_estimation_control_flags(const std::shared_ptr<ros2_mscl::srv::SetEstimationControlFlags::Request> req, std::shared_ptr<ros2_mscl::srv::SetEstimationControlFlags::Response> res);
    void get_estimation_control_flags(const std::shared_ptr<ros2_mscl::srv::GetEstimationControlFlags::Request> req, std::shared_ptr<ros2_mscl::srv::GetEstimationControlFlags::Response> res);

    void set_dynamics_mode(const std::shared_ptr<ros2_mscl::srv::SetDynamicsMode::Request> req, std::shared_ptr<ros2_mscl::srv::SetDynamicsMode::Response> res);
    void get_dynamics_mode(const std::shared_ptr<ros2_mscl::srv::GetDynamicsMode::Request> req, std::shared_ptr<ros2_mscl::srv::GetDynamicsMode::Response> res);

    void set_zero_angle_update_threshold(const std::shared_ptr<ros2_mscl::srv::SetZeroAngleUpdateThreshold::Request> req, std::shared_ptr<ros2_mscl::srv::SetZeroAngleUpdateThreshold::Response> res);
    void get_zero_angle_update_threshold(const std::shared_ptr<ros2_mscl::srv::GetZeroAngleUpdateThreshold::Request> req, std::shared_ptr<ros2_mscl::srv::GetZeroAngleUpdateThreshold::Response> res);
    
    void set_zero_velocity_update_threshold(const std::shared_ptr<ros2_mscl::srv::SetZeroVelocityUpdateThreshold::Request> req, std::shared_ptr<ros2_mscl::srv::SetZeroVelocityUpdateThreshold::Response> res);
    void get_zero_velocity_update_threshold(const std::shared_ptr<ros2_mscl::srv::GetZeroVelocityUpdateThreshold::Request> req, std::shared_ptr<ros2_mscl::srv::GetZeroVelocityUpdateThreshold::Response> res);
    
    void set_tare_orientation(const std::shared_ptr<ros2_mscl::srv::SetTareOrientation::Request> req, std::shared_ptr<ros2_mscl::srv::SetTareOrientation::Response> res);
    
    void commanded_vel_zupt(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void commanded_ang_rate_zupt(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  
    void set_accel_noise(const std::shared_ptr<ros2_mscl::srv::SetAccelNoise::Request> req, std::shared_ptr<ros2_mscl::srv::SetAccelNoise::Response> res);
    void get_accel_noise(const std::shared_ptr<ros2_mscl::srv::GetAccelNoise::Request> req, std::shared_ptr<ros2_mscl::srv::GetAccelNoise::Response> res);

    void set_gyro_noise(const std::shared_ptr<ros2_mscl::srv::SetGyroNoise::Request> req, std::shared_ptr<ros2_mscl::srv::SetGyroNoise::Response> res);
    void get_gyro_noise(const std::shared_ptr<ros2_mscl::srv::GetGyroNoise::Request> req, std::shared_ptr<ros2_mscl::srv::GetGyroNoise::Response> res);

    void set_mag_noise(const std::shared_ptr<ros2_mscl::srv::SetMagNoise::Request> req, std::shared_ptr<ros2_mscl::srv::SetMagNoise::Response> res);
    void get_mag_noise(const std::shared_ptr<ros2_mscl::srv::GetMagNoise::Request> req, std::shared_ptr<ros2_mscl::srv::GetMagNoise::Response> res);

    void set_gyro_bias_model(const std::shared_ptr<ros2_mscl::srv::SetGyroBiasModel::Request> req, std::shared_ptr<ros2_mscl::srv::SetGyroBiasModel::Response> res);
    void get_gyro_bias_model(const std::shared_ptr<ros2_mscl::srv::GetGyroBiasModel::Request> req, std::shared_ptr<ros2_mscl::srv::GetGyroBiasModel::Response> res);

    void set_accel_bias_model(const std::shared_ptr<ros2_mscl::srv::SetAccelBiasModel::Request> req, std::shared_ptr<ros2_mscl::srv::SetAccelBiasModel::Response> res);
    void get_accel_bias_model(const std::shared_ptr<ros2_mscl::srv::GetAccelBiasModel::Request> req, std::shared_ptr<ros2_mscl::srv::GetAccelBiasModel::Response> res);

    void set_gravity_adaptive_vals(const std::shared_ptr<ros2_mscl::srv::SetGravityAdaptiveVals::Request> req, std::shared_ptr<ros2_mscl::srv::SetGravityAdaptiveVals::Response> res);
    void get_gravity_adaptive_vals(const std::shared_ptr<ros2_mscl::srv::GetGravityAdaptiveVals::Request> req, std::shared_ptr<ros2_mscl::srv::GetGravityAdaptiveVals::Response> res);

    void set_mag_adaptive_vals(const std::shared_ptr<ros2_mscl::srv::SetMagAdaptiveVals::Request> req, std::shared_ptr<ros2_mscl::srv::SetMagAdaptiveVals::Response> res);
    void get_mag_adaptive_vals(const std::shared_ptr<ros2_mscl::srv::GetMagAdaptiveVals::Request> req, std::shared_ptr<ros2_mscl::srv::GetMagAdaptiveVals::Response> res);

    void set_mag_dip_adaptive_vals(const std::shared_ptr<ros2_mscl::srv::SetMagDipAdaptiveVals::Request> req, std::shared_ptr<ros2_mscl::srv::SetMagDipAdaptiveVals::Response> res);
    void get_mag_dip_adaptive_vals(const std::shared_ptr<ros2_mscl::srv::GetMagDipAdaptiveVals::Request> req, std::shared_ptr<ros2_mscl::srv::GetMagDipAdaptiveVals::Response> res);
    
    void external_heading_update(const std::shared_ptr<ros2_mscl::srv::ExternalHeadingUpdate::Request> req, std::shared_ptr<ros2_mscl::srv::ExternalHeadingUpdate::Response> res);

    void set_relative_position_reference(const std::shared_ptr<ros2_mscl::srv::SetRelativePositionReference::Request> req, std::shared_ptr<ros2_mscl::srv::SetRelativePositionReference::Response> res);
    void get_relative_position_reference(const std::shared_ptr<ros2_mscl::srv::GetRelativePositionReference::Request> req, std::shared_ptr<ros2_mscl::srv::GetRelativePositionReference::Response> res);

    void device_settings(const std::shared_ptr<ros2_mscl::srv::DeviceSettings::Request> req, std::shared_ptr<ros2_mscl::srv::DeviceSettings::Response> res);
    
    /*
    void velocity_zupt_callback(const std_msgs::Bool& state);
    void vel_zupt();
    
    void ang_zupt_callback(const std_msgs::Bool& state);
    void ang_zupt();    

    void external_gps_time_callback(const sensor_msgs::TimeReference& time);
    */
  private:


  //Convience for printing packet stats
  void print_packet_stats();

  //Variables/fields
  std::unique_ptr<mscl::InertialNode> m_inertial_device;

  
  double m_timer_update_rate_hz;

  //Info for converting to the ENU frame
  bool m_use_enu_frame;
  tf2::Matrix3x3 m_t_ned2enu;

  //Flag for using device timestamp instead of PC received time
  bool m_use_device_timestamp;

  //Packet Counters (valid, timeout, and checksum errors)
  uint32_t m_imu_valid_packet_count;
  uint32_t m_gnss_valid_packet_count[NUM_GNSS];
  uint32_t m_filter_valid_packet_count;
  uint32_t m_rtk_valid_packet_count;

  uint32_t m_imu_timeout_packet_count;
  uint32_t m_gnss_timeout_packet_count[NUM_GNSS];
  uint32_t m_filter_timeout_packet_count;

  uint32_t m_imu_checksum_error_packet_count;
  uint32_t m_gnss_checksum_error_packet_count[NUM_GNSS];
  uint32_t m_filter_checksum_error_packet_count;


  //Data field storage
  //IMU
  float m_curr_imu_mag_x;
  float m_curr_imu_mag_y;
  float m_curr_imu_mag_z;

  mscl::Vector m_curr_ahrs_quaternion;

  //FILTER
  double m_gps_leap_seconds;

  double m_curr_filter_pos_lat;
  double m_curr_filter_pos_long;
  double m_curr_filter_pos_height;

  float m_curr_filter_vel_north;
  float m_curr_filter_vel_east;
  float m_curr_filter_vel_down;

  mscl::Vector m_curr_filter_quaternion;

  float m_curr_filter_roll;
  float m_curr_filter_pitch;
  float m_curr_filter_yaw;

  float m_curr_filter_angular_rate_x;
  float m_curr_filter_angular_rate_y;
  float m_curr_filter_angular_rate_z;

  float m_curr_filter_pos_uncert_north;
  float m_curr_filter_pos_uncert_east;
  float m_curr_filter_pos_uncert_down;

  float m_curr_filter_vel_uncert_north;
  float m_curr_filter_vel_uncert_east;
  float m_curr_filter_vel_uncert_down;

  float m_curr_filter_att_uncert_roll;
  float m_curr_filter_att_uncert_pitch;
  float m_curr_filter_att_uncert_yaw;

 
  //Services
  
  rclcpp::Service<ros2_mscl::srv::SetTareOrientation>::SharedPtr m_set_tare_orientation_service;
  rclcpp::Service<ros2_mscl::srv::SetComplementaryFilter>::SharedPtr m_set_complementary_filter_service;
  rclcpp::Service<ros2_mscl::srv::GetComplementaryFilter>::SharedPtr m_get_complementary_filter_service;
  rclcpp::Service<ros2_mscl::srv::SetSensor2VehicleRotation>::SharedPtr m_set_sensor2vehicle_rotation_service;
  rclcpp::Service<ros2_mscl::srv::GetSensor2VehicleRotation>::SharedPtr m_get_sensor2vehicle_rotation_service;
  rclcpp::Service<ros2_mscl::srv::SetSensor2VehicleOffset>::SharedPtr m_set_sensor2vehicle_offset_service;
  rclcpp::Service<ros2_mscl::srv::GetSensor2VehicleOffset>::SharedPtr m_get_sensor2vehicle_offset_service;
  rclcpp::Service<ros2_mscl::srv::GetSensor2VehicleTransformation>::SharedPtr m_get_sensor2vehicle_transformation_service;
  
  rclcpp::Service<ros2_mscl::srv::SetAccelBias>::SharedPtr m_set_accel_bias_service;
  rclcpp::Service<ros2_mscl::srv::GetAccelBias>::SharedPtr m_get_accel_bias_service;
  
  rclcpp::Service<ros2_mscl::srv::SetGyroBias>::SharedPtr m_set_gyro_bias_service;
  rclcpp::Service<ros2_mscl::srv::GetGyroBias>::SharedPtr m_get_gyro_bias_service;
  
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_gyro_bias_capture_service;
  rclcpp::Service<ros2_mscl::srv::SetHardIronValues>::SharedPtr m_set_hard_iron_values_service;
  rclcpp::Service<ros2_mscl::srv::GetHardIronValues>::SharedPtr m_get_hard_iron_values_service;
  rclcpp::Service<ros2_mscl::srv::GetSoftIronMatrix>::SharedPtr m_get_soft_iron_matrix_service;
  rclcpp::Service<ros2_mscl::srv::SetSoftIronMatrix>::SharedPtr m_set_soft_iron_matrix_service;
  rclcpp::Service<ros2_mscl::srv::SetConingScullingComp>::SharedPtr m_set_coning_sculling_comp_service;
  rclcpp::Service<ros2_mscl::srv::GetConingScullingComp>::SharedPtr m_get_coning_sculling_comp_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_reset_filter_service;
  rclcpp::Service<ros2_mscl::srv::SetEstimationControlFlags>::SharedPtr m_set_estimation_control_flags_service;
  rclcpp::Service<ros2_mscl::srv::GetEstimationControlFlags>::SharedPtr m_get_estimation_control_flags_service;
  rclcpp::Service<ros2_mscl::srv::InitFilterEuler>::SharedPtr m_init_filter_euler_service;
  rclcpp::Service<ros2_mscl::srv::InitFilterHeading>::SharedPtr m_init_filter_heading_service;
  rclcpp::Service<ros2_mscl::srv::SetHeadingSource>::SharedPtr m_set_heading_source_service;
  rclcpp::Service<ros2_mscl::srv::GetHeadingSource>::SharedPtr m_get_heading_source_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_commanded_vel_zupt_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_commanded_ang_rate_zupt_service;
  rclcpp::Service<ros2_mscl::srv::SetAccelNoise>::SharedPtr m_set_accel_noise_service;
  rclcpp::Service<ros2_mscl::srv::GetAccelNoise>::SharedPtr m_get_accel_noise_service;
  rclcpp::Service<ros2_mscl::srv::SetGyroNoise>::SharedPtr m_set_gyro_noise_service;
  rclcpp::Service<ros2_mscl::srv::GetGyroNoise>::SharedPtr m_get_gyro_noise_service;
  rclcpp::Service<ros2_mscl::srv::SetMagNoise>::SharedPtr m_set_mag_noise_service;
  rclcpp::Service<ros2_mscl::srv::GetMagNoise>::SharedPtr m_get_mag_noise_service;
  rclcpp::Service<ros2_mscl::srv::SetAccelBiasModel>::SharedPtr m_set_accel_bias_model_service;
  rclcpp::Service<ros2_mscl::srv::GetAccelBiasModel>::SharedPtr m_get_accel_bias_model_service;
  rclcpp::Service<ros2_mscl::srv::SetGyroBiasModel>::SharedPtr m_set_gyro_bias_model_service;
  rclcpp::Service<ros2_mscl::srv::GetGyroBiasModel>::SharedPtr m_get_gyro_bias_model_service;
  rclcpp::Service<ros2_mscl::srv::SetMagAdaptiveVals>::SharedPtr m_set_mag_adaptive_vals_service;
  rclcpp::Service<ros2_mscl::srv::GetMagAdaptiveVals>::SharedPtr m_get_mag_adaptive_vals_service;
  rclcpp::Service<ros2_mscl::srv::SetMagDipAdaptiveVals>::SharedPtr m_set_mag_dip_adaptive_vals_service;
  rclcpp::Service<ros2_mscl::srv::GetMagDipAdaptiveVals>::SharedPtr m_get_mag_dip_adaptive_vals_service;
  rclcpp::Service<ros2_mscl::srv::SetGravityAdaptiveVals>::SharedPtr m_set_gravity_adaptive_vals_service;
  rclcpp::Service<ros2_mscl::srv::GetGravityAdaptiveVals>::SharedPtr m_get_gravity_adaptive_vals_service;
  rclcpp::Service<ros2_mscl::srv::SetZeroAngleUpdateThreshold>::SharedPtr m_set_zero_angle_update_threshold_service;
  rclcpp::Service<ros2_mscl::srv::GetZeroAngleUpdateThreshold>::SharedPtr m_get_zero_angle_update_threshold_service;
  rclcpp::Service<ros2_mscl::srv::SetZeroVelocityUpdateThreshold>::SharedPtr m_set_zero_velocity_update_threshold_service;
  rclcpp::Service<ros2_mscl::srv::GetZeroVelocityUpdateThreshold>::SharedPtr m_get_zero_velocity_update_threshold_service;
  rclcpp::Service<ros2_mscl::srv::SetReferencePosition>::SharedPtr m_set_reference_position_service;
  rclcpp::Service<ros2_mscl::srv::GetReferencePosition>::SharedPtr m_get_reference_position_service;
  rclcpp::Service<ros2_mscl::srv::SetDynamicsMode>::SharedPtr m_set_dynamics_mode_service;
  rclcpp::Service<ros2_mscl::srv::GetDynamicsMode>::SharedPtr m_get_dynamics_mode_service;
  rclcpp::Service<ros2_mscl::srv::DeviceSettings>::SharedPtr m_device_settings_service;
  rclcpp::Service<ros2_mscl::srv::ExternalHeadingUpdate>::SharedPtr m_external_heading_service;
  rclcpp::Service<ros2_mscl::srv::SetRelativePositionReference>::SharedPtr m_set_relative_position_reference_service;
  rclcpp::Service<ros2_mscl::srv::GetRelativePositionReference>::SharedPtr m_get_relative_position_reference_service;
 
  //IMU Publishers
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr m_mag_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<ros2_mscl_msgs::msg::GPSCorrelationTimestampStamped>::SharedPtr m_gps_corr_pub = nullptr;

  //GNSS Publishers
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_gnss_pub[NUM_GNSS] = {nullptr};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr m_gnss_odom_pub[NUM_GNSS] = {nullptr};
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::TimeReference>::SharedPtr m_gnss_time_pub[NUM_GNSS] = {nullptr};
  rclcpp_lifecycle::LifecyclePublisher<ros2_mscl_msgs::msg::GNSSAidingStatus>::SharedPtr m_gnss_aiding_status_pub[NUM_GNSS] = {nullptr};

  //RTK Data publisher
  rclcpp_lifecycle::LifecyclePublisher<ros2_mscl_msgs::msg::RTKStatus>::SharedPtr m_rtk_pub = nullptr;
  
  //Filter Publishers
  rclcpp_lifecycle::LifecyclePublisher<ros2_mscl_msgs::msg::FilterStatus>::SharedPtr m_filter_status_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<ros2_mscl_msgs::msg::FilterHeading>::SharedPtr m_filter_heading_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<ros2_mscl_msgs::msg::FilterHeadingState>::SharedPtr m_filter_heading_state_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr m_filter_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr m_filtered_imu_pub = nullptr;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr m_filter_relative_pos_pub = nullptr;

  //Device Status Publisher
  //rclcpp::Publisher<>::SharedPtr m_device_status_pub;
   
  //ZUPT subscribers
  //rclcpp::Subscriber m_filter_vel_state_sub;
  //rclcpp::Subscriber m_filter_ang_state_sub;

  //External GNSS subscriber
  //rclcpp::Subscriber m_external_gps_time_sub;

  //Timers
  rclcpp::TimerBase::SharedPtr m_main_loop_timer;
  rclcpp::TimerBase::SharedPtr m_device_status_timer;
    

  //IMU Messages
  sensor_msgs::msg::Imu           m_imu_msg;
  sensor_msgs::msg::MagneticField m_mag_msg;
  ros2_mscl_msgs::msg::GPSCorrelationTimestampStamped m_gps_corr_msg;

  //GNSS Messages
  sensor_msgs::msg::NavSatFix      m_gnss_msg[NUM_GNSS];
  nav_msgs::msg::Odometry          m_gnss_odom_msg[NUM_GNSS];
  sensor_msgs::msg::TimeReference  m_gnss_time_msg[NUM_GNSS];
  ros2_mscl_msgs::msg::GNSSAidingStatus m_gnss_aiding_status_msg[NUM_GNSS];

  //RTK Messages
  ros2_mscl_msgs::msg::RTKStatus   m_rtk_msg;
 
  //Filter Messages
  nav_msgs::msg::Odometry                 m_filter_msg;
  sensor_msgs::msg::Imu                   m_filtered_imu_msg;
  nav_msgs::msg::Odometry                 m_filter_relative_pos_msg;
  ros2_mscl_msgs::msg::FilterStatus       m_filter_status_msg;
  ros2_mscl_msgs::msg::FilterHeadingState m_filter_heading_state_msg;
  ros2_mscl_msgs::msg::FilterHeading      m_filter_heading_msg;

  //Device Status Message
  ros2_mscl_msgs::msg::Status m_device_status_msg;
 
  //Frame ids
  std::string m_imu_frame_id;
  std::string m_gnss_frame_id[NUM_GNSS];
  std::string m_filter_frame_id;
  std::string m_filter_child_frame_id;
 
  //Topic strings
  std::string m_velocity_zupt_topic;
  std::string m_angular_zupt_topic;
  std::string m_external_gps_time_topic;
  
  //Publish data flags
  bool m_publish_imu;
  bool m_publish_gps_corr;
  bool m_publish_gnss[NUM_GNSS];
  bool m_publish_gnss_aiding_status[NUM_GNSS];
  bool m_publish_filter;
  bool m_publish_filter_relative_pos;
  bool m_publish_rtk;

  //ZUPT, angular ZUPT topic listener variables
  bool m_angular_zupt;
  bool m_velocity_zupt;
  
  bool m_vel_still;
  bool m_ang_still;
  
  //Static covariance vectors
  std::vector<double> m_imu_linear_cov;
  std::vector<double> m_imu_angular_cov;
  std::vector<double> m_imu_orientation_cov;

  // Update rates
  int m_imu_data_rate;
  int m_gnss_data_rate[NUM_GNSS];
  int m_filter_data_rate;

  //Gnss antenna offsets
  std::vector<double> m_gnss_antenna_offset[NUM_GNSS];

  //Various settings variables
  clock_t m_start;
  uint8_t m_com_mode;
  float   m_field_data[3];
  float   m_soft_iron[9];
  float   m_soft_iron_readback[9];
  float   m_angles[3];
  float   m_heading_angle;
  float   m_readback_angles[3];
  float   m_noise[3];
  float   m_beta[3];
  float   m_readback_beta[3];
  float   m_readback_noise[3];
  float   m_offset[3];
  float   m_readback_offset[3];
  double  m_reference_position_command[3];
  double  m_reference_position_readback[3];
  uint8_t m_dynamics_mode;

  //Raw data file parameters
  bool          m_raw_file_enable;
  bool          m_raw_file_include_support_data;
  std::ofstream m_raw_file;
  }; //Microstrain class


  // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
  extern "C"
#endif
  {

#ifdef __cplusplus
  }
#endif

} // namespace Microstrain

#endif  // _MICROSTRAIN_3DM_GX5_45_H
