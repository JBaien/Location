#include "lidar_pointcloud/transform.h"
#include <lidar_pointcloud/pointcloudXYZIR.h>
#include <lidar_pointcloud/organized_cloudXYZIR.h>

namespace lidar_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new lidar_rawdata::RawData),
    first_rcfg_call(true),
    diagnostics_(node, private_nh, node_name)
  {
    boost::optional<lidar_pointcloud::Calibration> calibration = data_->setup(private_nh);
    if(calibration)
    {
      ROS_DEBUG_STREAM("Calibration file loaded.");
      config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
    }
    else
    {
      ROS_ERROR_STREAM("Could not load calibration file!");
    }

    config_.target_frame = config_.fixed_frame = "lidar";
    tf_ptr_ = boost::make_shared<tf::TransformListener>();

    if(config_.organize_cloud)
    {
      container_ptr = boost::shared_ptr<OrganizedCloudXYZIR>(
          new OrganizedCloudXYZIR(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                  config_.num_lasers, data_->scansPerPacket(), tf_ptr_));
    }
    else
    {
      container_ptr = boost::shared_ptr<PointcloudXYZIR>(
          new PointcloudXYZIR(config_.max_range, config_.min_range,
                              config_.target_frame, config_.fixed_frame,
                              data_->scansPerPacket(), tf_ptr_));
    }

    // advertise output point cloud (before subscribing to input data)
    output_ = node.advertise<sensor_msgs::PointCloud2>("lidar_points", 10);

    srv_ = boost::make_shared<dynamic_reconfigure::Server<TransformNodeCfg>> (private_nh);
    dynamic_reconfigure::Server<TransformNodeCfg>::CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to lidarScan packets using transform filter
    lidar_scan_.subscribe(node, "lidar_packets", 10);
    tf_filter_ptr_ = boost::shared_ptr<tf::MessageFilter<lidar_msgs::lidarScan> >(new tf::MessageFilter<lidar_msgs::lidarScan>(lidar_scan_, *tf_ptr_, config_.target_frame, 10));
    tf_filter_ptr_->registerCallback(boost::bind(&Transform::processScan, this, _1));
    private_nh.param<std::string>("fixed_frame", config_.fixed_frame, "odom");

    // Diagnostics
    diagnostics_.setHardwareID("lidar Transform");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("lidar_points", diagnostics_, FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10), TimeStampStatusParam()));

  }
  
  void Transform::reconfigure_callback(lidar_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");
    data_->setParameters(config.min_range, config.max_range, config.view_direction, config.view_width);
    config_.target_frame = tf::resolve(tf_prefix_, config.frame_id);
    ROS_INFO_STREAM("Target frame ID now: " << config_.target_frame);
    config_.min_range = config.min_range;
    config_.max_range = config.max_range;

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    if(first_rcfg_call || config.organize_cloud != config_.organize_cloud)
    {
      first_rcfg_call = false;
      config_.organize_cloud = config.organize_cloud;
      if(config_.organize_cloud)
      {
        ROS_INFO_STREAM("Using the organized cloud format...");
        container_ptr = boost::shared_ptr<OrganizedCloudXYZIR>(
            new OrganizedCloudXYZIR(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                    config_.num_lasers, data_->scansPerPacket()));
      }
      else
      {
        container_ptr = boost::shared_ptr<PointcloudXYZIR>(
            new PointcloudXYZIR(config_.max_range, config_.min_range, config_.target_frame, config_.fixed_frame,
                                data_->scansPerPacket()));
      }
    }
    container_ptr->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
  }

  void
    Transform::processScan(const lidar_msgs::lidarScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0) return;  // avoid much work

    boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);

    // allocate a point cloud with same time and frame ID as raw data
    container_ptr->setup(scanMsg);

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size()-1; ++i)
    {
      container_ptr->computeTransformation(scanMsg->packets[i].stamp);
      data_->unpack(scanMsg->packets[i], *container_ptr,i,scanMsg->header.stamp);
    } 
    // last packet
    size_t i = scanMsg->packets.size()-1;
    container_ptr->computeTransformation(scanMsg->packets[i].stamp);
    data_->unpack(scanMsg->packets[i], *container_ptr,1000,scanMsg->header.stamp);

    // publish the accumulated cloud message
    output_.publish(container_ptr->finishCloud());

    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }

} // namespace lidar_pointcloud
