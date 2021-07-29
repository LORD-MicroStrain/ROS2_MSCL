## Description

IMPORTANT:  This is a beta release of the ROS2 driver!  Please report any issues via the [Github issues page](https://github.com/LORD-MicroStrain/ROS2_MSCL/issues)


This is the ROS2 Interface (driver) software for inertial sensors compatible with the [Microstrain Communication Library (MSCL)](https://github.com/LORD-MicroStrain/MSCL).

MSCL is developed by [LORD Sensing - Microstrain](http://microstrain.com) in Williston, VT. 


## Build Instructions

#### MSCL

IMPORTANT: You will need to copy /usr/share/c++mscl/libmscl.so to /usr/lib manually for now as the ROS2 build environment cannot locate it.  We are looking into this.

MSCL pre-built packages can be found here: [MSCL Packages](https://github.com/LORD-MicroStrain/MSCL/releases/tag/v62.0.0)

We do our best to keep ROS2-MSCL up-to-date with the latest MSCL changes, but sometimes there is a delay. The currently supported version of MSCL is [v62.0.0](https://github.com/LORD-MicroStrain/MSCL/releases/tag/v62.0.0)

Install instructions can be found here: [How to Use MSCL](https://github.com/LORD-MicroStrain/MSCL/blob/master/HowToUseMSCL.md#linux)

If you choose to install MSCL at a location other than /usr/share, [CMakeLists.txt](https://github.com/LORD-MicroStrain/ROS2-MSCL/blob/master/CMakeLists.txt) will need to be updated with the install path.

#### Building from source
1. Install ROS2 and create a workspace: [Configuring Your ROS2 Environment](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html)

2. Move the entire ROS2-MSCL folder (ros_mscl and mscl_msgs) to the your_workspace/src directory.

3. Build your workspace:
        
        cd ~/your_workspace
        colcon make
        source ~/your_workspace/install/setup.bash
   The source command may need to be run in each terminal prior to launching a ROS node.

#### Launch the node and publish data
The following command will launch the driver. Keep in mind each instance needs to be run in a separate terminal.
            
        ros2 launch ros2_mscl microstrain_launch.py

Some optional launch parameters:
- name: namespace the node will publish messages to, default: gx5
- port: serial port name to connect to the device over, default: /dev/ttyACM0
- baudrate: baud rate to open the connection with, default: 115200
- imu_rate: sample rate for IMU data (hz), default: 100
    
To check published topics:
        
    ros2 topic list

This driver is implemented as a lifecycle node.  Upon running, the node will be in the unconfigured state and no interaction has occurred with the device.  The node must be transitioned as follows for data to be available:

- transition to configure state: 

        ros2 lifecycle set /gx5 configure

- transition to active state: 

        ros2 lifecycle set /gx5 activate

You can stop data from streaming by putting the device into the "deactivate" state.  Both the "cleanup" and "shutdown" states will disconnect from the device and close the raw data log file (if enabled.)


## License
ROS2-MSCL is released under the MIT License - see the `LICENSE` file in the source distribution.

Copyright (c)  2021, Parker Hannifin Corp.

