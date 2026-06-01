#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "lidar_pointcloud/transform.h"

namespace lidar_pointcloud
{
  class TransformNodelet: public nodelet::Nodelet
  {
  public:

    TransformNodelet() {}
    ~TransformNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Transform> tf_;
  };

  /** @brief Nodelet initialization. */
  void TransformNodelet::onInit()
  {
    tf_.reset(new Transform(getNodeHandle(), getPrivateNodeHandle(), getName()));
  }

} // namespace lidar_pointcloud 
PLUGINLIB_EXPORT_CLASS(lidar_pointcloud::TransformNodelet, nodelet::Nodelet)
