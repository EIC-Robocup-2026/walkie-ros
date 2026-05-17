/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_ros2.h"
#include <signal.h>

// Global pointer to access node for signal handler
UnitreeLidarSDKNode* g_node_ptr = nullptr;

// Signal handler for graceful shutdown
void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        if (g_node_ptr && g_node_ptr->lsdk_) {
            std::cout << "\n[INFO] Stopping lidar rotation before shutdown..." << std::endl;
            g_node_ptr->lsdk_->stopLidarRotation();
        }
        rclcpp::shutdown();
    }
}

int main(int argc, char *argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<UnitreeLidarSDKNode>();
  g_node_ptr = node.get();
  
  // Register signal handlers for graceful shutdown
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  rclcpp::spin(node);
  
  // Ensure lidar is stopped before exiting
  if (node->lsdk_) {
    std::cout << "[INFO] Stopping lidar rotation..." << std::endl;
    node->lsdk_->stopLidarRotation();
  }
  
  rclcpp::shutdown();
  return 0;
}