#ifndef lidar_LASERSCAN_lidar_LASERSCAN_H
#define lidar_LASERSCAN_lidar_LASERSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <dynamic_reconfigure/server.h>
#include <lidar_laserscan/lidarLaserScanConfig.h>

namespace lidar_laserscan
{

class lidarLaserScan
{
public:
  lidarLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

private:
  boost::mutex connect_mutex_;
  void connectCb();
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  lidarLaserScanConfig cfg_;
  dynamic_reconfigure::Server<lidarLaserScanConfig> srv_;
  void reconfig(lidarLaserScanConfig& config, uint32_t level);

  unsigned int ring_count_;
};

}  // namespace lidar_laserscan

#endif  // lidar_LASERSCAN_lidar_LASERSCAN_H
