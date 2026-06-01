#ifndef lidar_pointcloud_CONVERT_H
#define lidar_pointcloud_CONVERT_H

#include <string>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/PointCloud2.h>
#include <lidar_pointcloud/rawdata.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_pointcloud/CloudNodeConfig.h>

namespace lidar_pointcloud
{
class Convert
{
  public:
    Convert(
        ros::NodeHandle node,
        ros::NodeHandle private_nh,
        std::string const & node_name = ros::this_node::getName());
    ~Convert() {}

  private:
    void callback(lidar_pointcloud::CloudNodeConfig &config, uint32_t level);
    void processScan(const lidar_msgs::lidarScan::ConstPtr &scanMsg);

    boost::shared_ptr<dynamic_reconfigure::Server<lidar_pointcloud::CloudNodeConfig> > srv_;

    boost::shared_ptr<lidar_rawdata::RawData> data_;
    ros::Subscriber lidar_scan_;
    ros::Publisher output_;

    boost::shared_ptr<lidar_rawdata::DataContainerBase> container_ptr_;

    boost::mutex reconfigure_mtx_;

    /// configuration parameters
    typedef struct
    {
      std::string target_frame;      ///< target frame
      std::string fixed_frame;       ///< fixed frame
      bool organize_cloud;           ///< enable/disable organized cloud structure
      double max_range;              ///< maximum range to publish
      double min_range;              ///< minimum range to publish
      uint16_t num_lasers;           ///< number of lasers
      int npackets;                  ///< number of packets to combine
    }
    Config;
    Config config_;
    bool first_rcfg_call;


  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};
}  // namespace lidar_pointcloud

#endif  // lidar_pointcloud_CONVERT_H
