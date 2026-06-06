
#include "tm32_parser.h"


#include <cmath>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <cstdlib>


namespace timoo {
namespace driver {
namespace tm32{

    static inline double get_current_time_in_seconds() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}


inline void setNaN(base::TimooPoint& point)
{
    point.x = nanf("");
    point.y = nanf("");
    point.z = nanf("");
    point.intensity = 0;  // Use 0 to represent invalid or undefined intensity;
}

inline float SQR(const float& val) { return val*val; }

void BlockData::Load(const unsigned char* buf, int unit_num)
{
    units_.resize(unit_num);

    // Channel units
    for (int i = 0; i < unit_num; ++i)
    {
        const unsigned char* p = buf + i * 3;
        units_[i].dist = *((unsigned short*)p);
        units_[i].intensity = *((unsigned char*)(p + 2)) * 1.0f;
    }
}


void PackageData::Load(const unsigned char* buf)
{

    int block_num = 12;
    int block_size = 100;
    int half_block_size = 50;
    int block_offset = 2;
    int unit_num = 32;

    // For blocks
    blocks_.clear();
    blocks_.reserve(block_num);

    int index = 0;
    float azimuth_diff = 20.0;//?
    float azimuth = 0;
    for (int i = 0; i < block_num; ++i)
    {
        // Calculate difference between current and next block's azimuth angle.
        azimuth = *((unsigned short*)(buf + index + block_offset)) ; // 转换为unsigned short类型的指针后读取其指向的值

        if (i < (block_num - 1))
        {
            float next_azimuth = *((unsigned short*)(buf + index + block_size + block_offset));

            float tmp_azimuth_diff = next_azimuth - azimuth;

            if(tmp_azimuth_diff > 0) //?7
                azimuth_diff = tmp_azimuth_diff;
            else
                azimuth_diff = tmp_azimuth_diff + 36000;
        }

        float block_duration = azimuth_diff;
        
        // update block data
        blocks_.emplace_back(new BlockData());
        blocks_.back()->set_angle(azimuth);
        blocks_.back()->set_block_duration(block_duration);
        blocks_.back()->Load(buf + index + block_offset + 2 , unit_num);

        index += block_size;
    }

    // time_ = rosTimeFromGpsTimestamp(buf + 1200); 
    // TODO: 临时添加为系统时间，最终应为根据对应的block解算的gps时间. 20231219.gongjie
    // sys_sec_ = get_current_time_in_seconds();
}


int TM32DataParser::Init(const base::DeviceConfig& config) {

    cfg_ = config;
   // std::cout << "[TM32DataParser] Init, lidar type: " << int(cfg_.lidar_type) << std::endl;    

    if (cfg_.cut_angle > 0.0) {
        cfg_.cut_angle = int((360.0 + cfg_.cut_angle) * 100) % 36000;
    }

    int num_lasers = 32;
    calibration_.laser_corrections.clear();
    calibration_.num_lasers = num_lasers;
    calibration_.distance_resolution_m = 0.0025;
    calibration_.laser_corrections.resize(num_lasers);
    for (int i = 0; i < num_lasers; i++) {
        calibration_.laser_corrections[i].laser_ring = i;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (size_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) 
    {
        float rotation = (ROTATION_RESOLUTION * rot_index) * M_PI / 180.0;
        cos_rot_table_[rot_index] = cosf(rotation);
        sin_rot_table_[rot_index] = sinf(rotation);
    }

    // For points level time calculation
    pkt_duration_ = TM32_SINGLE_FIRING * 32.0 * 12;
    block_duration_ = TM32_SINGLE_FIRING * 32.0;
    single_point_duration_ = TM32_SINGLE_FIRING;

    gps_status_ = false;

    return 0;
}


void TM32DataParser::collectPoints(const std::shared_ptr<PackageData>& package, const BlockData& block, const double& block_start_time)
{

    float azimuth = block.angle() ;
    float block_duration = block.block_duration();
    float x, y, z, intensity;

    std::vector<base::TimooPoint> block_points;

    float odd_azimuth = azimuth;
    // float even_azimuth = azimuth + azimuth_diff_two_planes_;
    float even_azimuth = azimuth;
     
    // From block buffer to PCL points
    for (int w = 0; w < TM32_SCANS_PER_FIRING; ++w)
    {
        // points_buffer_
        block_points.emplace_back();

        auto& unit = block.units()[w];
        auto& point = block_points.back(); 

        timoo_pointcloud::tm32::LaserCorrection &corrections = calibration_.laser_corrections[w];
 
        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
       if(corrections.laser_ring % 2 == 0 )
        {
           azimuth = even_azimuth;  
        //printf("=====%d\n",azm);
        }else
        {
           azimuth = odd_azimuth;
        }


        float distance = unit.dist * 0.005f; 
        if (unit.dist == 0 || distance > cfg_.max_distance || distance < cfg_.min_distance)
        {
            setNaN(point);
            point.ring_id = corrections.laser_ring;
            point.time = block_start_time + w * single_point_duration_ * 1e-3;
        }
        else
        {
            /** correct for the laser rotation as a function of timing during the firings **/
            float azimuth_corrected_f = azimuth + block_duration * (w * TM32_SINGLE_FIRING) / TM32_BLOCK_TDURATION;
            int azimuth_corrected = ((int)round(azimuth_corrected_f)+36000) % 36000;
        
            // convert polar coordinates to Euclidean XYZ 
            float distance = unit.dist * calibration_.distance_resolution_m; 
            distance += corrections.dist_correction; 
            
            float cos_vert_angle = corrections.cos_vert_correction; 
            float sin_vert_angle = corrections.sin_vert_correction; 
            float cos_rot_correction = corrections.cos_rot_correction; 
            float sin_rot_correction = corrections.sin_rot_correction; 
    
            // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b) 
            // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b) 
            float cos_rot_angle = 
              cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
              sin_rot_table_[azimuth_corrected] * sin_rot_correction;
            float sin_rot_angle = 
              sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
              cos_rot_table_[azimuth_corrected] * sin_rot_correction;
    
            float horiz_offset = corrections.horiz_offset_correction;
            float vert_offset = corrections.vert_offset_correction;
    
            // Compute the distance in the xy plane (w/o accounting for rotation)
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;
    
            // Calculate temporal X, use absolute value.
            float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
            // Calculate temporal Y, use absolute value
            float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
            if (xx < 0) xx=-xx;
            if (yy < 0) yy=-yy;
      
            // Get 2points calibration values,Linear interpolation to get distance
            // correction for X and Y, that means distance correction use
            // different value at different distance
            float distance_corr_x = 0;
            float distance_corr_y = 0;
            if (corrections.two_pt_correction_available) {
              distance_corr_x = 
                (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4) 
                + corrections.dist_correction_x;
              distance_corr_x -= corrections.dist_correction;
              distance_corr_y = 
                (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                + corrections.dist_correction_y;
              distance_corr_y -= corrections.dist_correction;
            }
    
            float distance_x = distance + distance_corr_x;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle ;
            x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
    
            float distance_y = distance + distance_corr_y;
            /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle ;
            y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
    
            // Using distance_y is not symmetric, but the velodyne manual
            // does this.
            /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
            z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;

            /** Intensity Calculation */
            float min_intensity = corrections.min_intensity;
            float max_intensity = corrections.max_intensity;
    
            intensity = unit.intensity;
            float focal_offset = 256 
                                * (1 - corrections.focal_distance / 13100) 
                                * (1 - corrections.focal_distance / 13100);
            float focal_slope = corrections.focal_slope;
            intensity += focal_slope * (std::abs(focal_offset - 256 * 
                SQR(1 - static_cast<float>(unit.dist)/65535)));
            intensity = (intensity < min_intensity) ? min_intensity : intensity;
            intensity = (intensity > max_intensity) ? max_intensity : intensity;


            /** Timestamp Calculation */
            double time = block_start_time + w * single_point_duration_ * 1e-3;

            /** Use standard ROS coordinate system (right-hand rule) */
            point.x = y;
            point.y = -x;
            point.z = z;
            point.intensity = intensity;
            point.distance = distance;
            point.azimuth = azimuth_corrected;
            point.ring_id = corrections.laser_ring;
            point.time = time; // 绝对时间，注意：偏移时间不能在此处修改，此处只能拿到packet时间，拿不到frame时间
        }
    }

    if (cfg_.organize_cloud) {
        for (int k = 0; k < TM32_SCANS_PER_FIRING; ++k) {
            pkt_data_->points.emplace_back(block_points[ TM32_CHANNEL_MAP[k] ]);
        }
    } else {
        pkt_data_->points.insert(pkt_data_->points.end(), block_points.begin(), block_points.end());
    }
}


void TM32DataParser::setFrameStartInfo(const double& timestamp)
{
    pkt_data_ = std::make_shared<base::TimooPacketData>();
    pkt_data_->points.reserve(33800);
    pkt_data_->is_last_packet = false;
    pkt_data_->timestamp = timestamp;
}


void TM32DataParser::setFrameEndInfo(const double& timestamp) 
{
    // if (pkt_data_->points.empty()) { return; }
    pkt_data_->is_last_packet = true;
    pkt_data_->cut_point_index = pkt_data_->points.size();    
    pkt_data_->cut_point_time = timestamp;
}

void TM32DataParser::correctTimestamp(const base::Time& cur_pkt_time) 
{
    if (is_first_pkt_) {
        last_time_ = cur_pkt_time;
        is_first_pkt_ = false;
        return;
    }

    pkt_duration_ = (cur_pkt_time - last_time_).toSec() * 1000.0;  // for precision, convert to milli second
    block_duration_ = pkt_duration_ / 24.0;
    single_point_duration_ = block_duration_ / 16.0;
}


// 获取当前时间（微秒）- 修正版本，返回相对于小时的微秒数
int64_t getCurrentMicroseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    
    // 计算总微秒数
    int64_t total_microseconds = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
    
    // 转换为相对于小时的微秒数（与GPS时间格式一致）
    int64_t hour_microseconds = total_microseconds % 3600000000LL; // 一小时 = 3600秒 = 3600000000微秒
    
    return hour_microseconds;
}


// 使用预先获取的时间戳进行检测
inline void checkTimeDifferenceWithTime(uint32_t timestamp) {
    
    // 定义静态变量记录上一次检测的时间，防止过于频繁的检测
    static int64_t last_check_time = 0;
    
    // 立即获取当前时间
    int64_t currentTime = getCurrentMicroseconds();
    
    // 如果距离上次检测时间小于200微秒，跳过本次检测
    if (last_check_time != 0) {
        int64_t time_gap = currentTime - last_check_time;
        if (time_gap < 600) {
            return; // 跳过检测，避免过于频繁
        }
    }
    
    // 更新上次检测时间
    last_check_time = currentTime;
    
    uint32_t timestamp_sec_down = (timestamp % 1000000 / 1000) * 1000 + (timestamp % 1000);
    if(timestamp_sec_down > 1000000)
    {
        std::cout << "[InputUDP] Invalid GPS timestamp, skipping time difference check." << timestamp_sec_down << std::endl;
        return;
    }
    else
    {
         uint32_t timestamp_mod = timestamp % 1000000;
         uint32_t currentTime_mod = currentTime % 1000000;
    
    int diff = abs(static_cast<int>(timestamp_mod) - static_cast<int>(currentTime_mod));
    
    // 简化：只有在实际需要时才保留环绕处理
    bool isTimeCircle = false;
    if (diff > 700000) {
        diff = 1000000 - diff;
        isTimeCircle = true;
    }

    if(diff > 100000) {
    // 格式化时间输出（参考datetest_825.cpp格式）
    std::string gps_time = std::to_string(timestamp / 60000000) + ":" + 
                            std::to_string(timestamp % 60000000 / 1000000) + ":" + 
                            std::to_string(timestamp % 1000000 / 1000) + ":" + 
                            std::to_string(timestamp % 1000);

    std::string current_time = std::to_string(currentTime / 60000000) + ":" +
                               std::to_string(currentTime % 60000000 / 1000000) + ":" +
                               std::to_string(currentTime % 1000000 / 1000) + ":" +
                               std::to_string(currentTime % 1000);

    std::cout << "[TM32Parser] Time difference detected!" << std::endl;
    std::cout << "GPS time: " << gps_time << " | Local time: " << current_time << std::endl;
    std::cout << "GPS(mod): " << timestamp_mod << " | Local(mod): " << currentTime_mod
              << " | Diff: " << diff << " μs" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    }
    }
   
}


base::Time TM32DataParser::extractPacketTime(const RawDataPtr raw_data) {

    static const int HOUR_TO_SEC = 3600;
    // nano second, from latest hour
    uint32_t timestamp = ((uint8_t)raw_data->d[1203] << 24) | ((uint8_t)raw_data->d[1202] << 16) | 
                         ((uint8_t)raw_data->d[1201] << 8) | (uint8_t)raw_data->d[1200]; 
    
    double system_time_sec = raw_data->time.toSec();

    base::Time stamp;
    uint32_t time_hour = (int)floor(system_time_sec) / HOUR_TO_SEC;

    stamp = base::Time((time_hour * HOUR_TO_SEC) + ((timestamp) / 1000000.0),
                                ((timestamp) % 1000000) * 1000); 

    
    // 使用接收时立即获取的时间戳进行检测
    //checkTimeDifferenceWithTime(timestamp);

    return stamp;
}


int TM32DataParser::packetDataParse(const RawDataPtr raw_data)
{
    // Get package data
    auto package = std::make_shared<PackageData>();

    base::Time packet_time = raw_data->time;
    if (cfg_.use_gps_clock && gps_status_) {
        // packet_time 更新为gps/ntp时间
        packet_time = extractPacketTime(raw_data);
        package->set_unix_sec(packet_time);
    } else {
        package->set_sys_sec(raw_data->time);
    }

    package->Load(raw_data->d.data());
    
    // Set frame start info
    setFrameStartInfo(packet_time.toSec());

    // Correct timestamp, for network transmission delay
    correctTimestamp(packet_time);

    // Parse package data
    double block_start_time = packet_time.toSec() - pkt_duration_ * 1e-3;
    for (BlockDataPtr block : package->blocks())
    {
        if (cfg_.cut_angle < 0.0 && block->angle() < last_angle_ ) {
            setFrameEndInfo(block_start_time);
        }

        if (cfg_.cut_angle > 0.0 && ((block->angle() >= cfg_.cut_angle && last_angle_ < cfg_.cut_angle)  
                                      || (cfg_.cut_angle <= block->angle() && block->angle() < last_angle_)
                                      || (block->angle() < last_angle_ && last_angle_ < cfg_.cut_angle)
                                    )) {
            setFrameEndInfo(block_start_time);
        }

        // Collect points from block
        collectPoints(package, *block, block_start_time);
        block_start_time += block_duration_ * 1e-3;
        
        // Record last info
        last_angle_ = block->angle();
    }

    // Record last info
    last_time_ = packet_time;
    // std::cout << "[TM32DataParser] packet time: " << packet_time.sec << "." << packet_time.nsec 
    //           << ", point num: " << pkt_data_->points.size() << std::endl;
    
    return 0;
}

void TM32DataParser::checkGPSStatus(const RawDataPtr raw_data) {
    if (raw_data->d[1000] == 0xaa && raw_data->d[1001] == 0x55) {
        gps_status_ = true;
    } else {
        gps_status_ = false; // 严格来说55 AA才为异常，此处将其它所有情况都判断为异常
    }
}

int TM32DataParser::statusDataParse(const RawDataPtr raw_data)
{
    // Check gps status, through every DIFOP packet
    checkGPSStatus(raw_data);

    if (calibrate_updated_) { return 0; }

    vertical_angle_list_.resize(32);
    for(int i = 0; i < 32; i++)
    {
        Float4Byte float4byte;
        for(int j = 0 ; j < 4 ; j++) 
        {
            float4byte.bytes[j] = raw_data->d[834 + 4*i + j];
        }

        //对垂直角度的有效性判断
        if(float4byte.value >= -18 + i && float4byte.value <= -14 + i)
        {
            vertical_angle_list_[TM32_CHANNEL_MAP[i]] = float4byte.value;  //校准参数无误，则读取实际校准值
        }
        else
        {
			vertical_angle_list_[TM32_CHANNEL_MAP[i]] = TM32_CHANNEL_TO_ANGLE_VALUE[i];   //校准参数有问题，则按照默认值
        }
    }

    // Update calibration params
    for(int i = 0; i < 32; i++){
      timoo_pointcloud::tm32::LaserCorrection &corrections = calibration_.laser_corrections[i];
      std::cout << "[TM32DataParser] laser " << i << " vertical angle: " << vertical_angle_list_[i] << std::endl;
      calibration_.laser_corrections[i].vert_correction = vertical_angle_list_[i] * M_PI /180;
      calibration_.laser_corrections[i].cos_vert_correction = cosf(calibration_.laser_corrections[i].vert_correction);
      calibration_.laser_corrections[i].sin_vert_correction = sinf(calibration_.laser_corrections[i].vert_correction);
      calibration_.laser_corrections[i].cos_rot_correction = cosf(0.0);
      calibration_.laser_corrections[i].sin_rot_correction = sinf(0.0);
    }

    // azimuth_diff_two_planes_ = (raw_data->d[1014] << 8) +  raw_data->d[1013];

    calibrate_updated_ = true;

    return 0;
}

void TM32DataParser::printVerticalCalibWarning() {
    // 获取当前时间
    auto now = std::chrono::steady_clock::now();
    // 计算与上次打印的时间差（秒）
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_vertical_calib_warning_time_
    ).count();
    
    // 如果超过1秒或首次打印，则输出提示
    if (elapsed >= 1) {
        std::cout << "[TM16DataParser] vertical angle calibration not update, please check status packet!" << std::endl;
        last_vertical_calib_warning_time_ = now;  // 更新最后打印时间
    }
}

int TM32DataParser::parse(const RawDataPtr raw_data) {
    if (raw_data->type == DataType::STATUS) {
        return statusDataParse(raw_data);
    } else if (raw_data->type == DataType::PACKET) {
        
        if (!calibrate_updated_) {
            printVerticalCalibWarning();
        }

        return packetDataParse(raw_data);
    } else {
        return -1;
    }

    return 0;
}



}

}

}