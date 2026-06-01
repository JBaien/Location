#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "lidar_laserscan/lidar_laserscan.h"

namespace lidar_laserscan
{

class LaserScanNodelet: public nodelet::Nodelet
{
public:
  LaserScanNodelet() {}
  ~LaserScanNodelet() {}

private:
  virtual void onInit()
  {
    node_.reset(new lidarLaserScan(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<lidarLaserScan> node_;
};

}  // namespace lidar_laserscan

PLUGINLIB_EXPORT_CLASS(lidar_laserscan::LaserScanNodelet, nodelet::Nodelet);
