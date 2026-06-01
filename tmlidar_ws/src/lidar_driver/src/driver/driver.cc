#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lidar_msgs/lidarScan.h>

#include "lidar_driver/driver.h"
#include <chrono>

namespace lidar_driver
{

  lidarDriver::lidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh, std::string const &node_name) : diagnostics_(node, private_nh, node_name)
  {
    // use private node handle to get parameters
    private_nh.param("frame_id", config_.frame_id, std::string("lidar"));
    std::string tf_prefix = tf::getPrefixParam(private_nh);
    
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);
    
    // get model name, validate string, determine packet rate
    private_nh.param("model", config_.model, std::string("lidar"));
    double packet_rate; // packet frequency (Hz)
    std::string model_full_name;

    if (config_.model == "lidar")
    {
      packet_rate = 840; 
      model_full_name = "lidar";
    }
    else
    {
      ROS_ERROR_STREAM("unknown lidar LIDAR model: " << config_.model);
      packet_rate = 2600.0;
    }
    std::string deviceName(std::string("lidar ") + model_full_name);

    private_nh.param("rpm", config_.rpm, 600.0);
    ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
    double frequency = (config_.rpm / 60.0); // expected Hz rate

    // default number of packets for each scan is a single revolution
    // (fractions rounded up)
    config_.npackets = (int)ceil(packet_rate / frequency);
    private_nh.getParam("npackets", config_.npackets);
    ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

    // if we are timestamping based on the first or last packet in the scan
    private_nh.param("timestamp_first_packet", config_.timestamp_first_packet, false);
    if (config_.timestamp_first_packet)
      ROS_INFO("Setting lidar scan start time to timestamp of first packet");

    std::string dump_file;
    private_nh.param("pcap", dump_file, std::string(""));

    double cut_angle;
    private_nh.param("cut_angle", cut_angle, -0.01);
    if (cut_angle < 0.0)
    {
      ROS_INFO_STREAM("Cut at specific angle feature deactivated.");
    }
    else if (cut_angle < (2 * M_PI))
    {
      ROS_INFO_STREAM("Cut at specific angle feature activated.Cutting lidar points always at "<< cut_angle << " rad.");
    }
    else
    {
      ROS_ERROR_STREAM("cut_angle parameter is out of range. Allowed range is "
                       << "between 0.0 and 2*PI or negative values to deactivate this feature.");
      cut_angle = -0.01;
    }

    // Convert cut_angle from radian to one-hundredth degree,
    // which is used in lidar packets
    config_.cut_angle = int((cut_angle * 360 / (2 * M_PI)) * 100);

    int udp_port;
    private_nh.param("port", udp_port, (int)DATA_PORT_NUMBER);

    // Initialize dynamic reconfigure
    srv_ = boost::make_shared<dynamic_reconfigure::Server<lidar_driver::
                                                              lidarNodeConfig>>(private_nh);
    dynamic_reconfigure::Server<lidar_driver::lidarNodeConfig>::
        CallbackType f;
    f = boost::bind(&lidarDriver::callback, this, _1, _2);
    srv_->setCallback(f); // Set callback function und call initially

    // initialize diagnostics
    diagnostics_.setHardwareID(deviceName);
    const double diag_freq = packet_rate / config_.npackets;
    diag_max_freq_ = diag_freq;
    diag_min_freq_ = diag_freq;
    ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("lidar_packets", diagnostics_,
                                          FrequencyStatusParam(&diag_min_freq_,
                                                               &diag_max_freq_,
                                                               0.1, 10),
                                          TimeStampStatusParam()));
    diag_timer_ = private_nh.createTimer(ros::Duration(0.2), &lidarDriver::diagTimerCallback, this);

    config_.enabled = true;

    // open lidar input device or file
    if (dump_file != "") // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new lidar_driver::InputPCAP(private_nh, udp_port,
                                             packet_rate, dump_file));
    }
    else
    {
      // read data from live socket
      input_.reset(new lidar_driver::InputSocket(private_nh, udp_port));
    }

    // raw packet output topic
    output_ = node.advertise<lidar_msgs::lidarScan>("lidar_packets", 10);

    last_azimuth_ = -1;

    //
    pre_length_ = 0;
    post_length_ = 0;
  }

  /** poll the device
 *
 *  @returns true unless end of file reached
 */
  bool lidarDriver::poll(void)
  {
    if (!config_.enabled)
    {
      // If we are not enabled exit once a second to let the caller handle
      // anything it might need to, such as if it needs to exit.
      ros::Duration(1).sleep();
      return true;
    }

    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lidar_msgs::lidarScanPtr scan(new lidar_msgs::lidarScan);

    // here modified  by zyl for split frame at cross 0 degree

    scan->packets.reserve(config_.npackets);
    if (pre_length_ > 0)
    {
      scan->packets.push_back(pre_split_packet_);
      post_length_ = pre_length_;
    }

    lidar_msgs::lidarPacket tmp_packet;
    int frame_length = 0;
    int st_break = 0;

    while (true)
    {
      // 1.wait net packet of fixed size
      while (true)
      {
        int rc = input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0)
          break; // got a full packet?
        if (rc < 0)
          return false; // end of file reached?
      }
      // 2.check packet

      uint16_t azimuth;
      for (size_t i = 0; i < 12; i++)
      {
        std::memcpy(&azimuth, &tmp_packet.data[100 * i + 2], sizeof(uint16_t));

        if (last_azimuth_ == -1)
        {
          last_azimuth_ = azimuth;
          continue;
        }

        int azimuth_diff = std::abs(azimuth - last_azimuth_);

        last_azimuth_ = azimuth;

        if (azimuth_diff > 1000)
        {
          // cross zero,split frame,
          pre_split_packet_.stamp = tmp_packet.stamp;
          std::memcpy(&pre_split_packet_.data[0], &tmp_packet.data[0], 1206);
          // length for next frame
          pre_length_ = 12 - i;
          if (i > 0)
          {
            // the last packet
            scan->packets.push_back(tmp_packet);
          }
          // to publish
          st_break = 1;
          break;
        }
        frame_length++;
      }
      // to publish
      if (st_break == 1)
        break;

      // normal
      scan->packets.push_back(tmp_packet);
    }

    // before publish,monitor
    // check sub time and size
    auto t1 = std::chrono::steady_clock::now();
    double t1_us = std::chrono::duration<double, std::micro>(t1.time_since_epoch()).count();
    size_t size_packets = scan->packets.size();
    uint16_t azi_first;
    uint16_t azi_end;
    uint16_t pos_head;
    uint16_t pos_end;

    pos_head = (12 - post_length_) * 100 + 2;
    pos_end = ((12 + 12 - pre_length_ - 1) % 12) * 100 + 2;

    std::memcpy(&azi_first, &scan->packets[0].data[pos_head], sizeof(uint16_t));

    std::memcpy(&azi_end, &scan->packets[size_packets - 1].data[pos_end], sizeof(uint16_t));

    //std::cout << "lidarDriver: " << std::to_string(t1_us)
    //          << ":" <<  std::to_string(size_packets)
    //          << ":" <<  std::to_string(frame_length)
    //          << ":" << std::to_string(azi_first)
    //         << ":" << std::to_string(azi_end)
    //          << std::endl;

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full lidar scan.");
    if (config_.timestamp_first_packet)
    {
      scan->header.stamp = scan->packets.front().stamp;
    }
    else
    {
      scan->header.stamp = scan->packets.back().stamp;
    }
    scan->header.frame_id = config_.frame_id;
    output_.publish(scan);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic_->tick(scan->header.stamp);
    diagnostics_.update();

    return true;
  }

  void lidarDriver::callback(lidar_driver::lidarNodeConfig &config,
                           uint32_t level)
  {
    ROS_INFO("Reconfigure Request");
    if (level & 1)
    {
      config_.time_offset = config.time_offset;
    }
    if (level & 2)
    {
      config_.enabled = config.enabled;
    }
  }

  void lidarDriver::diagTimerCallback(const ros::TimerEvent &event)
  {
    (void)event;
    // Call necessary to provide an error when no lidar packets are received
    diagnostics_.update();
  }

} // namespace lidar_driver
