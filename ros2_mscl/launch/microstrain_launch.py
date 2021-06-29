from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

# Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# Please consult your device's documentation for supported features

def generate_launch_description():
      return LaunchDescription([

           # ****************************************************************** 
           # Microstrain sensor node 
           # ****************************************************************** 

           LifecycleNode(
                  package    = "ros2_mscl",
                  executable = "ros2_mscl_node",
                  namespace  = '',
                  name       = "gx5",
                  parameters = [

                        {
                              # ****************************************************************** 
                              # General Settings 
                              # ****************************************************************** 
                              
                              "port"            : "/dev/ttyACM0",
                              "baudrate"        : 115200,
                              "debug"           : False,
                              "diagnostics"     : True,
                              "imu_frame_id"    : "sensor",
                              "gnss1_frame_id"  : "gnss1_antenna_wgs84",
                              "gnss2_frame_id"  : "gnss2_antenna_wgs84",
                              "filter_frame_id" : "sensor_wgs84",

                              # Controls if the driver outputs data with-respect-to ENU frame
                              #      false - position, velocity, and orientation are WRT the NED frame (native device frame)
                              #      true  - position, velocity, and orientation are WRT the ENU frame

                              "use_enu_frame" : False,

                              # Controls if the driver-defined setup is sent to the device
                              #      false - The driver will ignore the settings below and use the device's current settings
                              #      true  - Overwrite the current device settings with those listed below

                              "device_setup" : True,

                              # Controls if the driver-defined settings are saved
                              #      false - Do not save the settings
                              #      true  - Save the settings in the device's non-volatile memory

                              "save_settings" : False,

                              # Controls if the driver uses the device generated timestamp (if available) for timestamping messages
                              #      false - Use PC received time for timestamping
                              #      true  - Use device generated timestamp

                              "use_device_timestamp" : False,

                              # Controls if the driver creates a raw binary file
                              #      false - Do not create the file
                              #      true  - Create the file
                              #
                              #      Notes: 1) The filename will have the following format -
                              #                model_number "_" serial_number "_" datetime (year_month_day_hour_minute_sec) ".bin"
                              #                example: "3DM-GX5-45_6251.00001_20_12_01_01_01_01.bin"
                              #             2) This file is useful for getting support from the manufacturer

                              "raw_file_enable" : False,

                              # (GQ7 only) Controls if the driver requests additional factory support data to be included in the raw data file
                              #      false - Do not request the additional data
                              #      true  - Request the additional channels (please see notes below!)
                              #
                              #      Notes: **We recommend only enabling this feature when specifically requested by Microstrain.**
                              #      
                              #      Including this feature increases communication bandwidth requirements significantly... for serial data connections,
                              #      please ensure the baudrate is sufficient for the added data channels.     

                              "raw_file_include_support_data" : False,

                              # The directory to store the raw data file (no trailing '/')

                              "raw_file_directory" : "/home/your_name",


                              # ****************************************************************** 
                              # IMU Settings 
                              # ****************************************************************** 

                              "publish_imu"   : True,
                              "imu_data_rate" : 100,

                              # Static IMU message covariance values (the device does not generate these) 
                              # Since internally these are std::vector we need to use the rosparam tags 
                              "imu_orientation_cov" : [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                              "imu_linear_cov"      : [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                              "imu_angular_cov"     : [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],

                              # If enabled, this message can be used to validate time IMU time syncronzation with gps 
                              # It is most useful when using an external timesource and external PPS
                              # (see: filter_enable_external_gps_time_update) 

                              "publish_gps_corr" : False,

                              # ****************************************************************** 
                              # GNSS Settings (only applicable for devices with GNSS) 
                              # ****************************************************************** 

                              "publish_gnss1"   : True,
                              "gnss1_data_rate" : 2,

                              # Antenna #1 lever arm offset vector
                              #     For GQ7 - in the vehicle frame wrt IMU origin (meters)
                              #     For all other models - in the IMU frame wrt IMU origin (meters)
                              #     Note: Make this as accurate as possible for good performance

                              "gnss1_antenna_offset" : [0.0, -0.7, -1.0],

                              # GNSS2 settings are only applicable for multi-GNSS systems (e.g. GQ7) 
                              "publish_gnss2"   : True,
                              "gnss2_data_rate" : 2,

                              # Antenna #2 lever arm offset vector (In the vehicle frame wrt IMU origin (meters) )
                              #Note: Make this as accurate as possible for good performance 

                              "gnss2_antenna_offset" : [0.0, 0.7, -1.0],

                              # (GQ7 Only) Enable RTK dongle interface 
                              "rtk_dongle_enable" : False,

                              # ****************************************************************** 
                              # Kalman Filter Settings (only applicable for devices with a Kalman Filter) 
                              # ****************************************************************** 

                              "publish_filter"   : True,
                              "filter_data_rate" : 10,

                              # Sensor2vehicle frame transformation selector
                              #     0 = None, 1 = Euler Angles, 2 - matrix, 3 - quaternion
                              #     Notes: These are different ways of setting the same parameter in the device.
                              #            The different options are provided as a convenience.
                              #            Support for matrix and quaternion options is firmware version dependent (Quaternion not currently supported on GQ7)
                              #            Quaternion order is [i, j, k, w]

                              "filter_sensor2vehicle_frame_selector" : 1,

                              "filter_sensor2vehicle_frame_transformation_euler"      : [0.0, 0.0, 0.0],
                              "filter_sensor2vehicle_frame_transformation_matrix"     : [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                              "filter_sensor2vehicle_frame_transformation_quaternion" : [0.0, 0.0, 0.0, 1.0],

                               # Controls if the Kalman filter is reset after the settings are configured
                              "filter_reset_after_config" : True,

                              # Controls if the Kalman filter will auto-init or requires manual initialization
                              "filter_auto_init" : True,

                              # (All, except -10, and -15 products) Declination Source 1 = None, 2 = magnetic model, 3 = manual 
                              "filter_declination_source" : 2,
                              "filter_declination"        : 0.23,

                              # (All, except GQ7, -10, and -15 products) Heading Source 0 = None, 1 = magnetic, 2 = GNSS velocity (note: see manual for limitations)  
                              "filter_heading_source" : 1,

                              # (GX5 and previous,-45 models only) Dynamics Mode 1 = Portable (default), 2 = Automotive, 3 = Airborne (<2Gs), 4 = Airborne High G (<4Gs) 
                              "filter_dynamics_mode" : 1,

                              # ZUPT control 
                              "filter_velocity_zupt_topic" : "/moving_vel",
                              "filter_angular_zupt_topic"  : "/moving_ang",
                              "filter_velocity_zupt"       : True,
                              "filter_angular_zupt"        : True,

                              # (GQ7 full support, GX5-45 limited support) Adaptive filter settings
                              #      Adaptive level: 0 - off, 1 - Conservative, 2 = Moderate (default), 3 = agressive
                              #      Time limit: Max duration of measurement rejection prior to recovery, in milliseconds - default = 15000 

                              "filter_adaptive_level"         : 2,
                              "filter_adaptive_time_limit_ms" : 15000,

                              # (GQ7 only) Aiding measurement control 
                              "filter_enable_gnss_pos_vel_aiding"     : True,
                              "filter_enable_gnss_heading_aiding"     : True,
                              "filter_enable_altimeter_aiding"        : False,
                              "filter_enable_odometer_aiding"         : False,
                              "filter_enable_magnetometer_aiding"     : False,
                              "filter_enable_external_heading_aiding" : False,

                              #  External GPS Time Update Control
                              #      Notes:    filter_external_gps_time_topic should publish at no more than 1 Hz.
                              #                gps_leap_seconds should be updated to reflect the current number
                              #                of leap seconds.

                              "filter_enable_external_gps_time_update" : False,
                              "filter_external_gps_time_topic"         : "/external_gps_time",
                              "gps_leap_seconds"                       : 18.0,


                              #  (GQ7 only) GPIO Configuration
                              #    Notes:    For information on possible configurations and specific pin options,
                              #              refer to the MSCL MipNodeFeatures command, supportedGpioConfigurations.
                              #
                              #    GPIO Pins =
                              #    1 - GPIO1 (primary port pin 7) - Features = 0 - Unused, 1 - GPIO, 2 - PPS, 3 - Encoder
                              #    2 - GPIO2 (primary port pin 9) - Features = 0 - Unused, 1 - GPIO, 2 - PPS, 3 - Encoder
                              #    3 - GPIO3 (aux port pin 7)     - Features = 0 - Unused, 1 - GPIO, 2 - PPS, 3 - Encoder
                              #    4 - GPIO4 (aux port pin 9)     - Features = 0 - Unused, 1 - GPIO, 2 - PPS, 3 - Encoder
                              #
                              #    Feature:
                              #    0 - Unused   - Behaviors = 0 - unused
                              #    1 - GPIO     - Behaviors = 0 - unused, 1 - input, 2 - output low, 3 - output high
                              #    2 - PPS      - Behaviors = 0 - unused, 1 - input, 2 - output
                              #    3 - Encoder  - Behaviors = 0 - unused, 1 - enc A, 2 - enc B
                              #
                              #    GPIO Behavior:
                              #    0 - Unused
                              #    1 - Input
                              #    2 - Output Low
                              #    3 - Output High
                              #
                              #    PPS Behavior:
                              #    0 - Unused
                              #    1 - Input
                              #    2 - Output
                              #
                              #    Encoder Behavior:
                              #    0 - Unused
                              #    1 - Encoder A
                              #    2 - Encoder B
                              #
                              #    Pin Mode Bitfield:
                              #    1 - open drain
                              #    2 - pulldown
                              #    4 - pullup
      
                              "gpio1_feature"  : 0,
                              "gpio1_behavior" : 0,
                              "gpio1_pin_mode" : 0,

                              "gpio2_feature"  : 0,
                              "gpio2_behavior" : 0,
                              "gpio2_pin_mode" : 0,

                              "gpio3_feature"  : 0,
                              "gpio3_behavior" : 0,
                              "gpio3_pin_mode" : 0,

                              "gpio4_feature"  : 0,
                              "gpio4_behavior" : 0,
                              "gpio4_pin_mode" : 0,

                              "gpio_config"    : False,

                              # (GQ7 only) Filter Initialization control

                              #   Init Condition source =
                              #   0 - auto pos, vel, attitude (default)
                              #   1 - auto pos, vel, roll, pitch, manual heading
                              #   2 - auto pos, vel, manual attitude
                              #   3 - manual pos, vel, attitude
                              #
                              #   Auto-Heading alignment selector (note this is a bitfield, you can use more than 1 source) =
                              #   Bit 0 - Dual-antenna GNSS
                              #   Bit 1 - GNSS kinematic (requires motion, e.g. a GNSS velocity)
                              #   Bit 2 - Magnetometer
                              #
                              #   Reference frame =
                              #   1 - WGS84 Earth-fixed, earth centered (ECEF) position, velocity, attitude
                              #   2 - WGS84 Latitude, Longitude, height above ellipsoid position, NED velocity and attitude
      

                              "filter_init_condition_src"              : 0,
                              "filter_auto_heading_alignment_selector" : 1,
                              "filter_init_reference_frame"            : 2,
                              "filter_init_position" : [0.0, 0.0, 0.0],
                              "filter_init_velocity" : [0.0, 0.0, 0.0],
                              "filter_init_attitude" : [0.0, 0.0, 0.0],

                              # (GQ7 only) Relative Position Configuration
                              #   Reference frame =
                              #   1 - Relative ECEF position
                              #   2 - Relative LLH position
                              #
                              #   Reference position - Units provided by reference frame (ECEF - meters, LLH - deg, deg, meters)
                              #   Note: prior to firmware version 1.0.06 this command will fail for non-positive heights.  1.0.06 fixes this)
      
                              "publish_relative_position"      : False,
                              "filter_relative_position_frame" : 2,
                              "filter_relative_position_ref"   : [0, 0, 0.01],

                              # (GQ7 only) Speed Lever Arm Configuration
                              #   Lever Arm - In vehicle reference frame (meters)

                              "filter_speed_lever_arm" : [0.0, 0.0, 0.0],

                              # (GQ7 only) Wheeled Vehicle Constraint Control
                              #    Note: When enabled, the filter uses the assumption that velocity is constrained to the primary vehicle axis.
                              #          By convention, the primary vehicle axis is the vehicle X-axis
      
                              "filter_enable_wheeled_vehicle_constraint" : False,

                              # (GQ7 only) Vertical Gyro Constraint Control
                              #    Note: When enabled and no valid GNSS measurements are available, the filter uses the accelerometers to track
                              #          pitch and roll under the assumption that the sensor platform is not undergoing linear acceleration.
                              #          This constraint is useful to maintain accurate pitch and roll during GNSS signal outages.

                              "filter_enable_vertical_gyro_constraint" : False,

                              # (GQ7 only) GNSS Antenna Calibration Control
                              #Note: When enabled, the filter will enable lever arm error tracking, up to the maximum offset specified in meters.

                              "filter_enable_gnss_antenna_cal"     : False,
                              "filter_gnss_antenna_cal_max_offset" : 0.1,

                              # (GQ7 only) PPS Source
                              #    PPS Source =
                              #    0 - Disabled
                              #    1 - Reciever 1 (default)
                              #    2 - Reciever 2
                              #    3 - Generated from system oscillator
                              #    4 - GPIO 1 (provided by external source if supported)
                              #    5 - GPIO 2 (provided by external source if supported)
                              #    6 - GPIO 3 (provided by external source if supported)
                              #    7 - GPIO 4 (provided by external source if supported)
      
                              "filter_pps_source" : 1,
                        }
                  ]
           ) #Node End
      ]) #LaunchDescription End
  

 
 
