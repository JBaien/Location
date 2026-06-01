#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "lidar_pointcloud/convert.h"

namespace lidar_pointcloud
{
  class CloudNodelet: public nodelet::Nodelet
  {
  public:

    CloudNodelet() {}
    ~CloudNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Convert> conv_;
  };

  /** @brief Nodelet initialization. */
  void CloudNodelet::onInit()
  {
    conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }

} // namespace lidar_pointcloud
PLUGINLIB_EXPORT_CLASS(lidar_pointcloud::CloudNodelet, nodelet::Nodelet)
