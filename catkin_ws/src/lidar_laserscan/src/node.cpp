#include <ros/ros.h>
#include "lidar_laserscan/lidar_laserscan.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_laserscan_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // create lidarLaserScan class
  lidar_laserscan::lidarLaserScan n(nh, nh_priv);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
