/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord ROS2 Inertial Driver Implementation File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>

#include "ros2_mscl/microstrain_3dm.hpp"
#include <tf2/LinearMath/Transform.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{

Microstrain::Microstrain() : rclcpp_lifecycle::LifecycleNode("ros2_mscl_node")
{
  //Initialize the helper classes
  if (!MicrostrainNodeBase::initialize(this))
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize base node");

  //Declare/Initialize Parameters
  this->declare_parameter("port",                  std::string("/dev/ttyACM0"));
  this->declare_parameter("baudrate",              115200);
  this->declare_parameter("device_setup",          false);
  this->declare_parameter("save_settings",         false);
  this->declare_parameter("use_device_timestamp",  false);
  this->declare_parameter("use_enu_frame",         false);

  //IMU
  this->declare_parameter("publish_imu",           false);
  this->declare_parameter("publish_gps_corr",      false);
  this->declare_parameter("imu_data_rate",         10);
  this->declare_parameter("imu_orientation_cov",   default_matrix);
  this->declare_parameter("imu_linear_cov",        default_matrix);
  this->declare_parameter("imu_angular_cov",       default_matrix);
  this->declare_parameter("imu_frame_id",          m_config.m_imu_frame_id);

  //GNSS 1/2
  this->declare_parameter("publish_gnss1",         false);
  this->declare_parameter("publish_gnss2",         false);
  this->declare_parameter("gnss1_data_rate",       1);
  this->declare_parameter("gnss2_data_rate",       1);
  this->declare_parameter("gnss1_antenna_offset",  default_vector);
  this->declare_parameter("gnss2_antenna_offset",  default_vector);
  this->declare_parameter("gnss1_frame_id",        m_config.m_gnss_frame_id[GNSS1_ID]);
  this->declare_parameter("gnss2_frame_id",        m_config.m_gnss_frame_id[GNSS2_ID]);

  //RTK Dongle configuration
  this->declare_parameter("rtk_dongle_enable", false);
  
  //Filter
  this->declare_parameter("publish_filter",             false);
  this->declare_parameter("filter_reset_after_config",  true);
  this->declare_parameter("filter_auto_init",           true);
  this->declare_parameter("filter_data_rate",           10);
  this->declare_parameter("filter_frame_id",            m_config.m_filter_frame_id);
  this->declare_parameter("publish_relative_position",  false);
  this->declare_parameter("filter_sensor2vehicle_frame_selector",                  0);
  this->declare_parameter("filter_sensor2vehicle_frame_transformation_euler",      default_vector);
  this->declare_parameter("filter_sensor2vehicle_frame_transformation_matrix",     default_matrix);
  this->declare_parameter("filter_sensor2vehicle_frame_transformation_quaternion", default_quaternion);

  this->declare_parameter("filter_initial_heading",          0.0);
  this->declare_parameter("filter_heading_source",           0x1);
  this->declare_parameter("filter_declination_source",       2);
  this->declare_parameter("filter_declination",              0.23);
  this->declare_parameter("filter_dynamics_mode",            1);
  this->declare_parameter("filter_pps_source",               1);
  this->declare_parameter("gps_leap_seconds",                18.0);
  this->declare_parameter("filter_angular_zupt",             false);
  this->declare_parameter("filter_velocity_zupt",            false);
  this->declare_parameter("filter_velocity_zupt_topic",      std::string("/moving_vel"));
  this->declare_parameter("filter_angular_zupt_topic",       std::string("/moving_ang"));
  this->declare_parameter("filter_external_gps_time_topic",  std::string("/external_gps_time"));
 
  //Additional GQ7 Filter
  this->declare_parameter("filter_adaptive_level" ,                   2);
  this->declare_parameter("filter_adaptive_time_limit_ms" ,           15000);
  this->declare_parameter("filter_enable_gnss_pos_vel_aiding",        true);
  this->declare_parameter("filter_enable_gnss_heading_aiding",        true);
  this->declare_parameter("filter_enable_altimeter_aiding",           false);
  this->declare_parameter("filter_enable_odometer_aiding",            false);
  this->declare_parameter("filter_enable_magnetometer_aiding",        false);
  this->declare_parameter("filter_enable_external_heading_aiding",    false);
  this->declare_parameter("filter_enable_external_gps_time_update",   false);
  this->declare_parameter("filter_enable_acceleration_constraint",    0);
  this->declare_parameter("filter_enable_velocity_constraint",        0);
  this->declare_parameter("filter_enable_angular_constraint",         0);
  this->declare_parameter("filter_init_condition_src",                0);
  this->declare_parameter("filter_auto_heading_alignment_selector",   0);
  this->declare_parameter("filter_init_reference_frame",              2);
  this->declare_parameter("filter_init_position",                     default_vector);   
  this->declare_parameter("filter_init_velocity",                     default_vector);
  this->declare_parameter("filter_init_attitude",                     default_vector);
  this->declare_parameter("filter_relative_position_frame",           2);
  this->declare_parameter("filter_relative_position_ref",             default_vector);   
  this->declare_parameter("filter_speed_lever_arm",                   default_vector);   
  this->declare_parameter("filter_enable_wheeled_vehicle_constraint", false);
  this->declare_parameter("filter_enable_vertical_gyro_constraint",   false);
  this->declare_parameter("filter_enable_gnss_antenna_cal",           false);
  this->declare_parameter("filter_gnss_antenna_cal_max_offset",       0.1);   

  //GPIO Configuration
  this->declare_parameter("gpio1_feature",   0);
  this->declare_parameter("gpio1_behavior",  0);
  this->declare_parameter("gpio1_pin_mode",  0);

  this->declare_parameter("gpio2_feature",   0);
  this->declare_parameter("gpio2_behavior",  0);
  this->declare_parameter("gpio2_pin_mode",  0);

  this->declare_parameter("gpio3_feature",   0);
  this->declare_parameter("gpio3_behavior",  0);
  this->declare_parameter("gpio3_pin_mode",  0);

  this->declare_parameter("gpio4_feature",   0);
  this->declare_parameter("gpio4_behavior",  0);
  this->declare_parameter("gpio4_pin_mode",  0);

  this->declare_parameter("gpio_config",     false);
  
  //Raw data file save
  this->declare_parameter("raw_file_enable",               false);
  this->declare_parameter("raw_file_include_support_data", false);
  this->declare_parameter("raw_file_directory",            std::string("."));
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_configure(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
 
  if(configure_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_activate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  if(activate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_deactivate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  if(deactivate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Cleanup State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_cleanup(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
 // Shutdown State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_shutdown(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::configure_node()
{
  ///////////////////////////////////////////////////////////////////////////
  //
  //Main loop setup
  ///
  ///////////////////////////////////////////////////////////////////////////
  try {
    RCLCPP_DEBUG(this->get_logger(), "Initializing base node");
    if (!MicrostrainNodeBase::configure(this))
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to configure node base");
      return false;
    }
  }
  catch(mscl::Error_Connection)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: Device Disconnected");
    return false;
  }

  catch(mscl::Error &e)
  {
    RCLCPP_FATAL(this->get_logger(), "Error: %s", e.what());
    return false;
  }


  return true;
} 


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::activate_node()
{
  //Start timer callbacks
  std::chrono::milliseconds timer_interval_ms(static_cast<int>(1.0/m_timer_update_rate_hz*1000.0));
  
  //Create and stop the wall timers for data parsing and device status
  m_main_loop_timer     = this->create_wall_timer(timer_interval_ms, std::bind(&MicrostrainNodeBase::parse_and_publish, this));

  RCLCPP_INFO(this->get_logger(), "Data Parsing timer started at <%f> hz", m_timer_update_rate_hz);
  RCLCPP_INFO(this->get_logger(), "Resuming the device data streams");

  //Resume the device
  m_config.m_inertial_device->resume();
  
  //Activate publishers
  if (m_publishers.m_device_status_pub)
    m_publishers.m_device_status_pub->on_activate();

  //IMU Publishers
  if(m_publishers.m_imu_pub)
    m_publishers.m_imu_pub->on_activate();
   
  if(m_publishers.m_mag_pub)
    m_publishers.m_mag_pub->on_activate();

  if(m_publishers.m_gps_corr_pub)
    m_publishers.m_gps_corr_pub->on_activate();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(m_publishers.m_gnss_pub[i])
      m_publishers.m_gnss_pub[i]->on_activate();
  
    if(m_publishers.m_gnss_odom_pub[i])
      m_publishers.m_gnss_odom_pub[i]->on_activate();

    if(m_publishers.m_gnss_time_pub[i])
      m_publishers.m_gnss_time_pub[i]->on_activate();

    if(m_publishers.m_gnss_aiding_status_pub[i])
      m_publishers.m_gnss_aiding_status_pub[i]->on_activate();
  }

  //RTK Data publisher
  if(m_publishers.m_rtk_pub)
    m_publishers.m_rtk_pub->on_activate();
  
  //Filter Publishers
  if(m_publishers.m_filter_status_pub)
    m_publishers.m_filter_status_pub->on_activate();

  if(m_publishers.m_filter_heading_pub)
    m_publishers.m_filter_heading_pub->on_activate();

  if(m_publishers.m_filter_heading_state_pub)
    m_publishers.m_filter_heading_state_pub->on_activate();

  if(m_publishers.m_filter_pub)
    m_publishers.m_filter_pub->on_activate();

  if(m_publishers.m_filtered_imu_pub)
    m_publishers.m_filtered_imu_pub->on_activate();

  if(m_publishers.m_filter_relative_pos_pub)
    m_publishers.m_filter_relative_pos_pub->on_activate();
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::deactivate_node()
{
  //Stop timer callbacks
  m_main_loop_timer->cancel(); 
  m_device_status_timer->cancel();

  RCLCPP_INFO(this->get_logger(), "Device set to idle");

  //Set the device to idle
  m_config.m_inertial_device->setToIdle();

  //Deactivate publishers
  if (m_publishers.m_device_status_pub)
    m_publishers.m_device_status_pub->on_deactivate();
 
  //IMU Publishers
  if(m_publishers.m_imu_pub)
    m_publishers.m_imu_pub->on_deactivate();
   
  if(m_publishers.m_mag_pub)
    m_publishers.m_mag_pub->on_deactivate();

  if(m_publishers.m_gps_corr_pub)
    m_publishers.m_gps_corr_pub->on_deactivate();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(m_publishers.m_gnss_pub[i])
      m_publishers.m_gnss_pub[i]->on_deactivate();
  
    if(m_publishers.m_gnss_odom_pub[i])
      m_publishers.m_gnss_odom_pub[i]->on_deactivate();

    if(m_publishers.m_gnss_time_pub[i])
      m_publishers.m_gnss_time_pub[i]->on_deactivate();

    if(m_publishers.m_gnss_aiding_status_pub[i])
      m_publishers.m_gnss_aiding_status_pub[i]->on_deactivate();
  }

  //RTK Data publisher
  if(m_publishers.m_rtk_pub)
    m_publishers.m_rtk_pub->on_deactivate();
  
  //Filter Publishers
  if(m_publishers.m_filter_status_pub)
    m_publishers.m_filter_status_pub->on_deactivate();

  if(m_publishers.m_filter_heading_pub)
    m_publishers.m_filter_heading_pub->on_deactivate();

  if(m_publishers.m_filter_heading_state_pub)
    m_publishers.m_filter_heading_state_pub->on_deactivate();

  if(m_publishers.m_filter_pub)
    m_publishers.m_filter_pub->on_deactivate();

  if(m_publishers.m_filtered_imu_pub)
    m_publishers.m_filtered_imu_pub->on_deactivate();

  if(m_publishers.m_filter_relative_pos_pub)
    m_publishers.m_filter_relative_pos_pub->on_deactivate();

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Shutdown/Cleanup Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::shutdown_or_cleanup_node()
{

  //Release the inertial node, if necessary
  if(m_config.m_inertial_device)
  {
    m_config.m_inertial_device->setToIdle();
    m_config.m_inertial_device->connection().disconnect();
  }

  //Close raw data file if enabled
  if(m_config.m_raw_file_enable)
  {
    m_config.m_raw_file.close();
  }

  //Release timers
  m_main_loop_timer.reset();
  m_device_status_timer.reset();

  //Release publishers
  if (m_publishers.m_device_status_pub)
    m_publishers.m_device_status_pub.reset();

  //IMU Publishers
  if(m_publishers.m_imu_pub)
    m_publishers.m_imu_pub.reset();
   
  if(m_publishers.m_mag_pub)
    m_publishers.m_mag_pub.reset();

  if(m_publishers.m_gps_corr_pub)
    m_publishers.m_gps_corr_pub.reset();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(m_publishers.m_gnss_pub[i])
      m_publishers.m_gnss_pub[i].reset();
  
    if(m_publishers.m_gnss_odom_pub[i])
      m_publishers.m_gnss_odom_pub[i].reset();

    if(m_publishers.m_gnss_time_pub[i])
      m_publishers.m_gnss_time_pub[i].reset();

    if(m_publishers.m_gnss_aiding_status_pub[i])
      m_publishers.m_gnss_aiding_status_pub[i].reset();
  }

  //RTK Data publisher
  if(m_publishers.m_rtk_pub)
    m_publishers.m_rtk_pub.reset();
  
  //Filter Publishers
  if(m_publishers.m_filter_status_pub)
    m_publishers.m_filter_status_pub.reset();

  if(m_publishers.m_filter_heading_pub)
    m_publishers.m_filter_heading_pub.reset();

  if(m_publishers.m_filter_heading_state_pub)
    m_publishers.m_filter_heading_state_pub.reset();

  if(m_publishers.m_filter_pub)
    m_publishers.m_filter_pub.reset();

  if(m_publishers.m_filtered_imu_pub)
    m_publishers.m_filtered_imu_pub.reset();

  if(m_publishers.m_filter_relative_pos_pub)
    m_publishers.m_filter_relative_pos_pub.reset();

  //Release services


  return true;
}

} // namespace Microstrain
