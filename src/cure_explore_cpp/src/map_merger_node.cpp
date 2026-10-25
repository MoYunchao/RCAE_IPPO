/**
 * @file map_merger_node.cpp
 * @brief Main entry point for map merger node
 */

#include "cure_explore_cpp/map_merger.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cure_explore_cpp::MapMerger>();

  RCLCPP_INFO(node->get_logger(), "Starting Map Merger (C++)...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
