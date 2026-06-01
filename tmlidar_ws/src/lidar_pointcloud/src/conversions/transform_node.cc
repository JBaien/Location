#include <ros/ros.h>
#include "lidar_pointcloud/transform.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_node");

  // create conversion class, which subscribes to raw data
  lidar_pointcloud::Transform transform(ros::NodeHandle(), ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
