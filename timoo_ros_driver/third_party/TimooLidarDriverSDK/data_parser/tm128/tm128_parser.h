
#pragma once

#include <cstdint>
#include <vector>
#include <memory>
#include <time.h>
#include <iostream>


#include "interface/base_parser.h"
#include "interface/base_time.h"
#include "common/common.h"
#include "common/device_config.h"
//#include "../tm16/calibration.h"

namespace timoo {
namespace driver {



/// ///////////////////////////////
/// Package Data
/// ///////////////////////////////

// Float4Byte, status packet data struct

#pragma pack(1)

struct LONGITUDEandLATITUDE                //longitude and latitude infomation 22bytes
{
	char LATITUDE[10];
	char LONGITUDE[10];
	char NorS;
	char WorE;
};

struct INTENSITYCALIBRATION{
	int8_t bvalue[64];
	uint8_t avalue[64];
	uint16_t seg[3];
	uint8_t Reserved[122];
};


struct SERIAL_NUMBER
{
	uint8_t	LIDAR_TYPE;

	uint8_t LIDAR_VERSION;

	uint8_t GET_YEAR;

	uint8_t GET_MONTH;

	uint16_t NUMBER;
};



struct UTC_TIME
{
	uint8_t year;                                                           //年
	uint8_t month;														     //月
	uint8_t day;															 //日
	uint8_t hour;															 //时
	uint8_t minutes;														 //分
	uint8_t second;														 //秒
	uint16_t millisec;                                                    //毫秒
	uint16_t microsec;													 //微秒
};


struct IP_NUMBER
{
	uint8_t Net_address_1;
	uint8_t Net_address_2;
	uint8_t Computer_address_1;
	uint8_t Computer_address_2;
};

struct ETH_REGISTER                                                          //以太网信息
{
	IP_NUMBER ip_src;
	IP_NUMBER ip_dest;
	uint8_t mac_addr[6];
	uint16_t port[6];
};

struct IMU_DATA{
	int16_t IMU_acceleration_x;
	int16_t IMU_acceleration_y;
	int16_t IMU_acceleration_z;
	int16_t IMU_angular_velocity_x;
	int16_t IMU_angular_velocity_y;
	int16_t IMU_angular_velocity_z;
};


struct TM_128_DIFOP{

  	uint8_t LidarHeader[8];                                                //DIFOP识别头 8bytes  
	uint8_t Motor_RotateSpeed[2];                                      //雷达转速 2bytes
	ETH_REGISTER Ethernet_info;                                        //Ethernet信息 26bytes  
	UTC_TIME Lidar_time;                                               //雷达时间
	uint8_t WorkModel;										//雷达工作模式
	uint8_t IsRotationorStatic;                                       //雷达旋转/静止

	SERIAL_NUMBER Serial_number;                                       //序列号 
	uint8_t Top_firmware_version[5];									//顶板固件版本号
	uint8_t Bottom_firmware_version[5];										//底板固件版本号   
	uint16_t Phrase_loc;                                             //电机锁相相位

	int8_t Distance_calibration[512];									//距离校准（预留）
	INTENSITYCALIBRATION Intensity_calibration;									//强度校准（预留）
	uint8_t VerticalAngle_calibration[128];									//垂直角度校准(预留）
	int16_t Azimuth_calibration;								//水平角度校准（预留）

	LONGITUDEandLATITUDE GPS_Position;//经纬度
	uint16_t BlockCoef[7];
	uint8_t GPS_state[2];	//GPS状态信息
	uint16_t DenoiseSeg[5];
	uint8_t Reserved1;
	int16_t BoardAzimuth;
	int8_t Plane4Azimuth[4];

	//IMU模块
	uint32_t IMU_Time_Start; //1us
	uint32_t IMU_Time_End;
	uint16_t IMU_Temp;
	IMU_DATA IMU_data[4];
	uint8_t IMU_Reserved[72];

	uint8_t Reserved2[87];                                              //预留55字节	
	uint8_t Tail[2];																		//帧尾

  };

  struct ChannelData
  {
    uint16_t distance;
    uint8_t reflectivity;
  };


    struct TM128_DataBlock
    {
        uint8_t Row_id;
        uint16_t Azimuth;
        uint8_t Azimuth_diff;
        ChannelData channeldata[32];
    };

    struct TM128_Timestamp
    {
        uint8_t datatime[6];
        uint32_t timestamp;
    };


    struct TM128_MSOP
    {
        uint8_t header[4];
        TM128_DataBlock datablock[12];
        //TM128_DataBlock datablock[9];
        TM128_Timestamp Timestamp;
        uint16_t GPS_States;
        uint16_t Pkt_Count;
        uint8_t Reserve[18];
        uint8_t factory[2];
    };

struct LidarData
{
	std::vector<std::vector<int> >distance;
	std::vector<std::vector<int> >intensity;
	std::vector<std::vector<double> >azimuth;//��λ��
	std::vector<std::vector<uint32_t>> mtimestamp;
	//TM 128线特殊字段
	std::vector<std::vector<int> > Row_id;
	std::vector<std::vector<int> > azimuth_diff; //两板间水平角度差值
};


#pragma pack()

// union Float4Byte{
//     uint8_t bytes[4];
//     float value;
// };

// // UnitData
// struct UnitData
// {
//     float dist;
//     float intensity;
// };

static int ChannelToAngleNoTM128[32] = { 15, 0, 17, 16, 1, 8, 25, 24, 9, 2, 19, 18, 3, 10, 27, 26, 11, 4, 21, 20, 5, 12, 29, 28, 13, 6, 23, 22, 7, 14, 31, 30 };
static float AngleNoToAngleValuetm128[4][32];
static float VerticalDefValueTM128[32] = { -17.88, -17.08, -16.406, -15.606, -14.911, -14.111, -13.396, -12.596, -11.863, -11.063, -10.314, -9.514, -8.751, -7.951,
-7.175, -6.375, -5.59, -4.79, -3.998, -3.198, -2.4, -1.6, -0.8, 0, 0.8, 1.6, 2.398, 3.198, 3.99, 4.79, 5.575, 6.375 };


static const double  DSR_TOFFSET = 3.072;                                      // [µs]

static const double  BAG_TOFFSET = 1179.648;




class TM128DataParser : public BaseParser {

public:
    TM128DataParser() = default;
    ~TM128DataParser() = default;
    int Init(const base::DeviceConfig& config) override;
    int parse(const RawDataPtr raw_data) override;


private:
    // Packet data parse, points data or gps data etc.
    int packetDataParse(const RawDataPtr raw_data);

    // Status data parse, device info
    int statusDataParse(const RawDataPtr raw_data);

    // Points collect
    void correctTimestamp(const base::Time& cur_pkt_time);
    // void collectPoints(const std::shared_ptr<PackageData>& package, const BlockData& block, 
    //                    const double& block_start_time);

    // Set frame info, start
    void setFrameStartInfo(const double& timestamp);

    // Set frame info, end
    void setFrameEndInfo(const double& timestamp);

    void sendpoint(LidarData lidardata);
    /** 
     * Vertical angel correction table
     */
    std::vector<float> vertical_angle_list_;
    bool vertical_calibrate_updated_ = false;

    /** 
     * Cut frame by angle
     */
    double last_angle_ = -1;

    /** 
     * Frame time info
     */
    bool is_first_pkt_ = true;
    base::Time last_time_;

    float azimuthDiff_;
    float plane4Azimuth_[4];

    double cosTheta[4][32] = { 0 };
    double sinTheta[4][32] = { 0 };

    /** 
     *  Time compensation
     */
    double pkt_duration_;
    double block_duration_;
    double single_point_duration_;
    bool isStart_;
    LidarData lidardata;
};


} // namespace driver
} // namespace timoo




