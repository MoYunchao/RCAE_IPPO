// Copyright 2025 SenseBeetle
// SPDX-License-Identifier: Apache-2.0

#include "map_postprocessor/map_postprocessor.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_postprocessor::MapPostprocessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
