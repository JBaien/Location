
#ifndef lidar_DRIVER_DRIVER_H
#define lidar_DRIVER_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <lidar_driver/input.h>
#include <lidar_driver/lidarNodeConfig.h>

namespace lidar_driver
{

class lidarDriver
{
public:
  lidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const & node_name = ros::this_node::getName());
  ~lidarDriver() {}

  bool poll(void);

private:
  // Callback for dynamic reconfigure
  void callback(lidar_driver::lidarNodeConfig &config, uint32_t level);
  // Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback(const ros::TimerEvent&event);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<lidar_driver::lidarNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int    npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    int cut_angle;                   // cutting angle in 1/100°
    double time_offset;              // time in seconds added to each lidar time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;
  }
  config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  int last_azimuth_;

  // added by zyl for split frame
  lidar_msgs::lidarPacket pre_split_packet_;
  int pre_length_;
  int post_length_;
  /* diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};

}  // namespace lidar_driver

#endif  // lidar_DRIVER_DRIVER_H
