#pragma once
#include <cstddef>
typedef signed char int8;         /* 8 bit signed */
typedef unsigned char uint8;      /* 8 bit unsigned */
typedef short int16;              /* 16 bit signed */
typedef unsigned short uint16;    /* 16 bit unsigned */
typedef int int32;                /* 32 bit signed */
typedef unsigned int uint32;      /* 32 bit unsigned */
typedef long int64;                /* 64 bit signed */
typedef unsigned long uint64;      /* 64 bit unsigned */

constexpr size_t LIDAR_LINE_MAX = 192;                
constexpr size_t ONELINE_POINTNUM = 1008;              // 一根线的点数
constexpr size_t MULTIPLIER = 100;                    // 源水平角倍率
constexpr size_t HANGLE_MAX = 360 * MULTIPLIER;

#pragma pack(push, 1)
// imu信息
struct ImuInfo
{
    // imu 时间戳
    uint32 TimeStamp;
    // x轴加速度
    uint32 X_AxisAcceleration;
    // y轴加速度
    uint32 Y_AxisAcceleration;
    // z轴加速度
    uint32 Z_AxisAcceleration;
    // x轴角速度
    uint32 X_AxisAngularVelocity;
    // y轴角速度
    uint32 Y_AxisAngularVelocity;
    // z轴角速度
    uint32 Z_AxisAngularVelocity;
};
#pragma pack(pop)

#pragma pack(push, 1)
// 功能安全
struct FunctionalSafety
{
    // 雷达状态
    uint8 RadarStatus;
    // 故障码
    uint16 FaultCode;
    // 故障计数器
    uint8 FaultCounter;
};
#pragma pack(pop)


#pragma pack(push, 1)
// 网络安全
struct NetworkSafety
{
    uint8 DigitalSignature[32];
};
#pragma pack(pop)

#pragma pack(push, 1)
// 数据校验
struct DataCheck
{
    uint16 Check;
};
#pragma pack(pop)


#pragma pack(push, 1)
struct ILineData {
    // 计时值(距离)
    uint16 Distance;
    // 反射率
    uint8 refelect;
};
#pragma pack(pop)

#pragma pack(push, 1)
// 包头
struct PackageHead
{
    // 帧头
    uint8 FrameHead[2];
    // 帧长
    uint16 FrameLength;
    // 设备类型
    uint8 DeviceType[2];
    // 协议主版本号
    uint8 ProtocolMainVersion;
    // 协议子版本
    uint8 ProtocolSonVersion;
};
#pragma pack(pop)

#pragma pack(push, 1)
// 数据头信息
struct DataHeadInfo
{
    // 圈号
    uint16 CircleNumber;
    // 总包数
    uint16 TotalPackages;
    // 包序号
    uint16 PackageSerialNumber;
    // 工作频率
    uint16 WorkFrequency;
    // 回波重数
    uint8 EchoMultiplicity;
    // 列通道数
    uint16 ColumnChannelNumbe;
    // 行通道数
    uint16 RowsChannelNumbe;
};
#pragma pack(pop)


#pragma pack(push, 1)
// 数据块信息
struct DataBlockInfo
{
    // 水平角度分辨率
    uint16 HorizontalAngleResolution;
    // 垂直角度分辨率
    uint16 VerticalAngleResolution;
    // 转镜号
    uint8 MirrorNo;
    // 电机转速
    uint16 MotorSpeed;
    // 预留
    uint8 Reserve[1];
    // 组包方式
    uint8 PackagingMethod;
    // 每包数据块数量
    uint16 EachDataBlockNum;
    // 每包有效数据块数量
    uint16 EachValidDataBlockNum;
    // 数据块内行通道数
    uint16 DataBlockRowsNum;
    // 数据块内列通道数
    uint16 DataBlockColsNum;
    // 距离分辨率
    uint8 RangeResolution;
};
#pragma pack(pop)

#pragma pack(push, 1)
// 时钟信息
struct ClockInfo
{
    // 时钟源
    uint8 ClockSource;
    // 时间戳-秒
    uint32 TimeStampSec;
    // 时间戳-纳秒
    uint32 TimeStampNanoSec;
};
#pragma pack(pop)

#pragma pack(push, 1)
// 信息标志
struct InfoFlag
{
    uint16 InfoFlag;

    // 获取imu数据信息字段定义 0-不包含IMU字段 1-包含imu字段但内容无效 2-该字段有效
    inline uint8 get_imu_data_info_define()
    {
        return InfoFlag & 0b11;
    }
    // 获取【功能安全】字段定义 0-不包含【功能安全】字段 1-包含【功能安全】字段但内容无效 2-该字段有效
    inline uint8 get_functional_safety_define()
    {
        return (InfoFlag & 0b1100) >> 2;
    }
    // 获取【网络安全】字段定义 0-不包含【网络安全】字段 1-包含【网络安全】字段但内容无效 2-该字段有效
    inline uint8 get_network_safety_define()
    {
        return (InfoFlag & 0b110000) >> 4;
    }
    // 获取【校验】字段定义 0-不包含【校验】字段 1-包含【校验全】字段但内容无效 2-该字段有效
    inline uint8 get_check_define()
    {
        return (InfoFlag & 0b11000000) >> 6;
    }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct DataMainPre
{
    // 包头信息
    PackageHead package_head;
    // 数据头信息
    DataHeadInfo data_head_info;
    // 数据块信息
    DataBlockInfo data_block_info;
    // 时钟信息
    ClockInfo clock_info;
    // 预留
    uint8 reserve[16];
    // 信息标志
    InfoFlag info_flag;
};
#pragma pack(pop)