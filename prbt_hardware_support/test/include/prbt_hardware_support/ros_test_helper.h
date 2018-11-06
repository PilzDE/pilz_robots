#ifndef PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H
#define PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H

#include <ros/ros.h>

namespace prbt_hardware_support
{

/**
 * @brief Blocks until a node defined by node_name comes up.
 * @param node_name
 * @param loop_frequency Frequency at which the system is checked for the node.
 */
inline void waitForNode(std::string node_name, double loop_frequency = 10.0)
{
  ROS_ERROR_STREAM("Waiting for Node " << node_name);
  std::vector<std::string> node_names;
  while (ros::master::getNodes(node_names) &&
         std::find(node_names.begin(), node_names.end(), node_name) == node_names.end())
  {
    ros::Rate(loop_frequency).sleep();
  }
  ROS_ERROR_STREAM("Node " << node_name << " found");
}

}  // namespace prbt_hardware_support


#endif // PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H
