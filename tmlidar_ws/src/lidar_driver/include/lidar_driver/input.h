#ifndef lidar_DRIVER_INPUT_H
#define lidar_DRIVER_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>

#include <ros/ros.h>
#include <lidar_msgs/lidarPacket.h>

namespace lidar_driver
{

static uint16_t DATA_PORT_NUMBER = 2555;      // default data port
static uint16_t POSITION_PORT_NUMBER = 8308;  // default position port

/** @brief lidar input base class */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);
  virtual ~Input() {}

  /** @brief Read one lidar packet.
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(lidar_msgs::lidarPacket *pkt,const double time_offset) = 0;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
  bool gps_time_;
};

/** @brief Live lidar input from socket. */
class InputSocket: public Input
{
public:
  InputSocket(ros::NodeHandle private_nh, uint16_t port = DATA_PORT_NUMBER);
  virtual ~InputSocket();

  virtual int getPacket(lidar_msgs::lidarPacket *pkt, const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  int sockfd_;
  in_addr devip_;
};


/** @brief lidar input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, lidar's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP: public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh,
            uint16_t port = DATA_PORT_NUMBER,
            double packet_rate = 0.0,
            std::string filename = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~InputPCAP();

  virtual int getPacket(lidar_msgs::lidarPacket *pkt, const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

}  // namespace lidar_driver

#endif  // lidar_DRIVER_INPUT_H
