/*

Copyright (c) 2017, Brian Bingham
Copyright (c)  2020, Parker Hannifin Corp
This code is licensed under MIT license (see LICENSE file for details)

*/

#include "microstrain_3dm.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
   
  rclcpp::executors::SingleThreadedExecutor exe;

  auto node = std::make_shared<Microstrain::Microstrain>();

  exe.add_node(node->get_node_base_interface());

  exe.spin();
  
  rclcpp::shutdown();

 return 0;
}
