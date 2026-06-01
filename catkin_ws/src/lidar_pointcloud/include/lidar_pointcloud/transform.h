#ifndef lidar_pointcloud_TRANSFORM_H
#define lidar_pointcloud_TRANSFORM_H

#include <string>
#include <ros/ros.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/PointCloud2.h>

#include <lidar_pointcloud/rawdata.h>
#include <lidar_pointcloud/pointcloudXYZIR.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_pointcloud/TransformNodeConfig.h>

namespace lidar_pointcloud
{
using TransformNodeCfg = lidar_pointcloud::TransformNodeConfig;

class Transform
{
public:
  Transform(
      ros::NodeHandle node,
      ros::NodeHandle private_nh,
      std::string const & node_name = ros::this_node::getName());
  ~Transform()
  {
  }

private:
  void processScan(const lidar_msgs::lidarScan::ConstPtr& scanMsg);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<lidar_pointcloud::TransformNodeConfig>> srv_;
  void reconfigure_callback(lidar_pointcloud::TransformNodeConfig& config, uint32_t level);

  const std::string tf_prefix_;
  boost::shared_ptr<lidar_rawdata::RawData> data_;
  message_filters::Subscriber<lidar_msgs::lidarScan> lidar_scan_;
  ros::Publisher output_;
  boost::shared_ptr<tf::MessageFilter<lidar_msgs::lidarScan>> tf_filter_ptr_;
  boost::shared_ptr<tf::TransformListener> tf_ptr_;

  /// configuration parameters
  typedef struct
  {
    std::string target_frame;  ///< target frame
    std::string fixed_frame;   ///< fixed frame
    bool organize_cloud;       ///< enable/disable organized cloud structure
    double max_range;          ///< maximum range to publish
    double min_range;          ///< minimum range to publish
    uint16_t num_lasers;       ///< number of lasers
  }
  Config;
  Config config_;

  bool first_rcfg_call;

  boost::shared_ptr<lidar_rawdata::DataContainerBase> container_ptr;

  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  boost::mutex reconfigure_mtx_;
};
}  // namespace lidar_pointcloud

#endif  // lidar_pointcloud_TRANSFORM_H
