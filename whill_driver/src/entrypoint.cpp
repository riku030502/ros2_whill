// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    entrypoint.cpp
 * @brief   Entrypoint of WHILL Node
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "whill_driver/whill_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<whill_driver::WhillNode>());
  rclcpp::shutdown();
  return 0;
}
