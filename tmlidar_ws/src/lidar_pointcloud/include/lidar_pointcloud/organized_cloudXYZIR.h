#ifndef lidar_pointcloud_ORGANIZED_CLOUDXYZIR_H
#define lidar_pointcloud_ORGANIZED_CLOUDXYZIR_H

#include <lidar_pointcloud/datacontainerbase.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>

namespace lidar_pointcloud
{
  class OrganizedCloudXYZIR : public lidar_rawdata::DataContainerBase
  {
  public:
    OrganizedCloudXYZIR(const double max_range, const double min_range, const std::string &target_frame,
                        const std::string &fixed_frame, const unsigned int num_lasers, const unsigned int scans_per_block,
                        boost::shared_ptr<tf::TransformListener> tf_ptr = boost::shared_ptr<tf::TransformListener>());

    virtual void newLine();

    virtual void setup(const lidar_msgs::lidarScan::ConstPtr &scan_msg);

    virtual void addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance, const float intensity,const float timestamp);

  private:
    sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity;
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;
    sensor_msgs::PointCloud2Iterator<float> iter_timestamp;
  };
} /* namespace lidar_pointcloud */
#endif // lidar_pointcloud_ORGANIZED_CLOUDXYZIR_H
