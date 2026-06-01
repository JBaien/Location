#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <lidar_pointcloud/rawdata.h>

namespace lidar_rawdata
{
  inline float SQR(float val) { return val * val; }


  RawData::RawData() {}

  /** Update parameters: conversions and update */
  void RawData::setParameters(double min_range, double max_range, double view_direction, double view_width)
  {
    config_.min_range = min_range;
    config_.max_range = max_range;

    //converting angle parameters into the lidar reference (rad)
    config_.tmp_min_angle = view_direction + view_width / 2;
    config_.tmp_max_angle = view_direction - view_width / 2;

    //computing positive modulo to keep theses angles into [0;2*M_PI]
    config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

    //converting into the hardware lidar ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion
    config_.min_angle = 100 * (2 * M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
    config_.max_angle = 100 * (2 * M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
    if (config_.min_angle == config_.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      config_.min_angle = 0;
      config_.max_angle = 36000;
    }
  }

  int RawData::scansPerPacket() const
  {
    if (calibration_.num_lasers == 16)
    {
      return BLOCKS_PER_PACKET * lidar_FIRINGS_PER_BLOCK * lidar_SCANS_PER_FIRING;
    }
    else
    {
      return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
    }
  }

  /** Set up for on-line operation. */
  boost::optional<lidar_pointcloud::Calibration> RawData::setup(ros::NodeHandle private_nh)
  {
    // get path to angles.config file for this device
    if (!private_nh.getParam("calibration", config_.calibrationFile))
    {
      ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

      // have to use something: grab unit test version as a default
      std::string pkgPath = ros::package::getPath("lidar_pointcloud");
      config_.calibrationFile = pkgPath + "/params/lidar.yaml";
    }

    //ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized)
    {
      ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
      return boost::none;
    }

    ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index)
    {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
    return calibration_;
  }

  /** Set up for offline operation */
  int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
  {

    config_.max_range = max_range_;
    config_.min_range = min_range_;
    ROS_INFO_STREAM("data ranges to publish: [" << config_.min_range << ", "<< config_.max_range << "]");

    config_.calibrationFile = calibration_file;

    //ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized)
    {
      ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
      return -1;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index)
    {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }
    return 0;
  }

  void RawData::unpack(const lidar_msgs::lidarPacket &pkt, DataContainerBase &data, int sn_packet,const ros::Time& scan_start_time)
  {
    using lidar_pointcloud::LaserCorrection;
    ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

    /** special parsing for the VLP16 **/
    if (calibration_.num_lasers == 16)
    {
      unpack_lidar(pkt, data, sn_packet,scan_start_time);
      return;
    }

    ROS_DEBUG_STREAM("num_lasers error: " << calibration_.num_lasers);
    return;
  }

  void RawData::unpack_lidar(const lidar_msgs::lidarPacket &pkt, DataContainerBase &data, int sn_packet,const ros::Time& scan_start_time)
  {
    //added by zyl 20191003
    int block_start = 0;
    int block_end = 12;

    float x, y, z;
    float azimuth;
    float intensity;
    int omega, alpha;
    float distance_resolution = 0.002f;
    float deepth;

    uint16_t pos_base;

    float TABLE_COMP[16][5] = {
        // v_ang,    h_ang_off, h_off, v_off, ring no         ch
        {-1500, 0.00, 0.021, 0.00506, 0},    //    0
        {100, 1.08, 0.021, -0.00915, 8},     //    1
        {-1300, 2.16, 0.021, 0.00506, 1},    //    2
        {300, 3.24, 0.021, -0.00915, 9},     //    3
        {-1100, 4.32, 0.021, 0.00506, 2},    //    4
        {500, 5.40, 0.021, -0.00915, 10},    //    5
        {-900, 6.48, 0.021, 0.00506, 3},     //    6
        {700, 7.56, 0.021, -0.00915, 11},    //    7
        {-700, 8.64, -0.021, 0.00915, 4},    //    8
        {900, 9.72, -0.021, -0.00506, 12},   //    9
        {-500, 10.80, -0.021, 0.00915, 5},   //   10
        {1100, 11.88, -0.021, -0.00506, 13}, //   11
        {-300, 12.96, -0.021, 0.00915, 6},   //   12
        {1300, 14.04, -0.021, -0.00506, 14}, //   13
        {-100, 15.12, -0.021, 0.00915, 7},   //   14
        {1500, 16.20, -0.021, -0.00506, 15}, //   15
    };

    //
    const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];
    float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();
    union two_bytes tmp;

    if (sn_packet == 0)
    {
      if (raw->blocks[0].rotation < 10000)
      {
        block_start = 0;
        block_end = 12;
      }
      else
      {
        for (int i = 0; i < 11; i++)
        {
          if (std::abs(raw->blocks[i + 1].rotation - raw->blocks[i].rotation) > 1000)
          {
            block_start = i + 1;
            block_end = 12;
          }
        }
      }
      // here monitor the head azmith
      uint16_t azi_first;
      azi_first = raw->blocks[block_start].rotation;

      //std::cout << "rawdata az start: " << std::to_string(block_start)
      //          << ":" <<  std::to_string(azi_first)
      //          << std::endl;
    }
    else if (sn_packet == 1000)
    {
      block_start = 0;
      block_end = 12;
      for (int i = 0; i < 11; i++)
      {
        if (std::abs(raw->blocks[i + 1].rotation - raw->blocks[i].rotation) > 1000)
        {
          block_end = i + 1;
        }
      }
      // here monitor the end azmith
      uint16_t azi_end;
      azi_end = raw->blocks[block_end - 1].rotation;
    }
    else
    {
      block_start = 0;
      block_end = 12;
    }
    //                            =12
    for (int block = block_start; block < block_end; block++)
    {

      // Calculate difference between current and next block's azimuth angle.
      // 取一个数据块的角度
      // X=R * cos(ω) * sin(α) + h_off * cos(α)
      // Y=R * cos(ω) * cos(α) - h_off * sin(α)
      // Z=R * sin(ω) + v_off
      //取得角度
      azimuth = (float)(raw->blocks[block].rotation);
      pos_base = 0;
      for (int ch = 0; ch < 16; ch++)
      {
        //union two_bytes tmp;

        tmp.bytes[0] = raw->blocks[block].data[pos_base];     //深度数据 低位
        tmp.bytes[1] = raw->blocks[block].data[pos_base + 1]; //深度数据 高位
                                                              //取得距离
        deepth = tmp.uint * distance_resolution;
        //取得反射率
        intensity = raw->blocks[block].data[pos_base + 2];
        //垂直角
        omega = ((int)(TABLE_COMP[ch][0] + 36000)) % 36000;
        //水平角
        alpha = ((int)(azimuth + TABLE_COMP[ch][1])) % 36000;


        if (deepth < config_.min_range)
        {
          x = NAN;
          y = NAN;
          z = NAN;
          deepth = NAN;
          intensity = 0;
        }
        else
        {
          x = deepth * cos_rot_table_[omega] * sin_rot_table_[alpha] + TABLE_COMP[ch][2] * cos_rot_table_[alpha];
          y = deepth * cos_rot_table_[omega] * cos_rot_table_[alpha] - TABLE_COMP[ch][2] * sin_rot_table_[alpha];
          z = deepth * sin_rot_table_[omega] + TABLE_COMP[ch][3];
        }
        
        /** Use standard ROS coordinate system (right-hand rule) */
        //float x_coord = y;
        //float y_coord = -x;
        //float z_coord = z;
        // x = int(x * 1000) / 1000.0f;
        //y = int(y * 1000) / 1000.0f;
        //z = int(z * 1000) / 1000.0f;
        double Treal = time_diff_start_to_this_packet + ((51*(block*2))+(3*ch))/1000000.0;
        data.addPoint(y, -x, z, (int)TABLE_COMP[ch][4], alpha, deepth, intensity,Treal);
        pos_base += 3;
      }
      data.newLine();

      azimuth += 18;
      pos_base = 48;
      for (int ch = 0; ch < 16; ch++)
      {
        //union two_bytes tmp;

        tmp.bytes[0] = raw->blocks[block].data[pos_base];     //depth low
        tmp.bytes[1] = raw->blocks[block].data[pos_base + 1]; //depth high
        //距离
        deepth = tmp.uint * distance_resolution;
        //反射率
        intensity = raw->blocks[block].data[pos_base + 2];
        //垂直角
        omega = ((int)(TABLE_COMP[ch][0] + 36000)) % 36000;
        //水平角
        alpha = ((int)(azimuth + TABLE_COMP[ch][1])) % 36000;
        if (deepth < config_.min_range)
        {
          x = NAN;
          y = NAN;
          z = NAN;
          deepth = NAN;
          intensity = 0;
        }
        else
        {
          x = deepth * cos_rot_table_[omega] * sin_rot_table_[alpha] + TABLE_COMP[ch][2] * cos_rot_table_[alpha];
          y = deepth * cos_rot_table_[omega] * cos_rot_table_[alpha] - TABLE_COMP[ch][2] * sin_rot_table_[alpha];
          z = deepth * sin_rot_table_[omega] + TABLE_COMP[ch][3];
        }
        

        /** Use standard ROS coordinate system (right-hand rule) */
        //float x_coord = y;
        //float y_coord = -x;
        //float z_coord = z;
        //x = int(x * 1000) / 1000.0f;
        //y = int(y * 1000) / 1000.0f;
        //z = int(z * 1000) / 1000.0f;
        //这里一定要是double，float的精度不够
        double Treal = time_diff_start_to_this_packet + ((51*(block*2+1))+(3*ch))/1000000.0;
        data.addPoint(y, -x, z, (int)TABLE_COMP[ch][4], alpha, deepth, intensity,Treal);
        pos_base += 3;
      }
      data.newLine();
    }
  }

} // namespace lidar_rawdata
