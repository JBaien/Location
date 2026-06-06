#pragma once
#include <vector>
#include "protocol.h"
#include <cassert>
#include <cstring> 
#include <iostream>
#include "common/point_type.h"

namespace timoo {
namespace driver {

#pragma pack(push, 1)
//单点信息结构体
class OnePointInfo
{
public:
    int32 index = 0;                              // 此点在1圈数据中的位置
    int32 line = 0;                               // 线号 1-144
    int32 line_id = 0;                            // 协议里的原始角度
    char Mirror = 0;                              // 转镜号
    float PointHAngle = 0;                        // 计算后的水平角度 (0~360)
    float PointVAngle = 0;                        // 计算后的垂直角度 (0~360)
    float distance_m = 0;                         // 距离
    float x = 0;					              // x轴坐标，单位米
    float y = 0;					              // y轴坐标，单位米
    float z = 0;					              // z轴坐标，单位米
    uint8 reflectivity = 0;                       // 反射率
    std::uint64_t Time = 0;
    unsigned char EchoType = 0;                   // 回波类型 1- 最强 2最后


    unsigned char confidence = 0;                 // 置信度标志
    int32 SrcHAngle = 0;                          // 协议里的原始角度
    unsigned short motor_speed;                   // 电机转速
    unsigned int distance_mm = 0;                 // 距离(毫米)
    float CosRideSin = 0;                         // 根据角度计算的cos * sin
    float CosRidCos = 0;                          // 根据角度计算的cos * cos
    float SinRidThousandth = 0;                   // 根据角度计算的sin值

};
#pragma pack(pop)//恢复对齐状态


static float Single459_VerticalChannels[] = {
    12.47104231, 12.33297464, 12.19513719, 12.05752781, 11.92014438, 11.78298474, 11.64604673, 11.50932819,
    11.37282694, 11.2365408, 11.10046758, 10.96460507, 10.82895108, 10.69350338, 10.55825975, 10.42321796,
    10.28837576, 10.15373091, 10.01928115, 9.885024208, 9.750957822, 9.617079708, 9.483387576, 9.349879132,
    9.216552072, 9.083404084, 8.950432851, 8.817636045, 8.685011334, 8.552556378, 8.420268828, 8.28814633,
    8.156186522, 8.024387036, 7.892745496, 7.761259519, 7.629926717, 7.498744693, 7.367711046, 7.236823367,
    7.106079242, 6.975476248, 6.845011958, 6.71468394, 6.584489753, 6.454426952, 6.324493086, 6.194685698,
    6.065002326, 5.935440502, 5.805997751, 5.676671596, 5.547459553, 5.418359132, 5.289367839, 5.160483175,
    5.031702637, 4.903023716, 4.774443899, 4.645960668, 4.5175715, 4.389273871, 4.26106525, 4.132943101,
    4.004904886, 3.876948064, 3.749070087, 3.621268407, 3.49354047, 3.36588372, 3.238295596, 3.110773536,
    2.983314973, 2.855917339, 2.728578061, 2.601294566, 2.474064276, 2.346884611, 2.219752989, 2.092666827,
    1.965623537, 1.838620532, 1.711655222, 1.584725014, 1.457827315, 1.330959529, 1.204119061, 1.077303312,
    0.950509684, 0.823735576, 0.696978388, 0.570235518, 0.443504363, 0.316782321, 0.190066789, 0.063355162,
    -0.063355162, -0.190066789, -0.316782321, -0.443504363, -0.570235518, -0.696978388, -0.823735576,
    -0.950509684, -1.077303312, -1.204119061, -1.330959529, -1.457827315, -1.584725014, -1.711655222,
    -1.838620532, -1.965623537, -2.092666827, -2.219752989, -2.346884611, -2.474064276, -2.601294566,
    -2.728578061, -2.855917339, -2.983314973, -3.110773536, -3.238295596, -3.36588372 , -3.49354047 ,
    -3.621268407, -3.749070087, -3.876948064, -4.004904886, -4.132943101, -4.26106525 , -4.389273871,
    -4.5175715  , -4.645960668, -4.774443899, -4.903023716, -5.031702637, -5.160483175, -5.289367839,
    -5.418359132, -5.547459553, -5.676671596, -5.805997751, -5.935440502, -6.065002326, -6.194685698,
    -6.324493086, -6.454426952, -6.584489753, -6.71468394 , -6.845011958, -6.975476248, -7.106079242,
    -7.236823367, -7.367711046, -7.498744693, -7.629926717, -7.761259519, -7.892745496, -8.024387036,
    -8.156186522, -8.28814633 , -8.420268828, -8.552556378, -8.685011334, -8.817636045, -8.950432851,
    -9.083404084, -9.216552072, -9.349879132, -9.483387576, -9.617079708, -9.750957822, -9.885024208,
    -10.01928115, -10.15373091, -10.28837576, -10.42321796, -10.55825975, -10.69350338, -10.82895108,
    -10.96460507, -11.10046758, -11.2365408 , -11.37282694, -11.50932819, -11.64604673, -11.78298474,
    -11.92014438, -12.05752781, -12.19513719, -12.33297464, -12.47104231,
};

static float DegreeToRadian = 3.1415926 / 180;



//#define USE_VECTOR


struct PointSizeCn
{
    size_t firstSize = 0;
    size_t LastSize = 0;
};


class CustomerAnalysisBase
{
protected:
    int size = 0;
    int nextCircleNo = 0;

    std::vector<std::vector<uint8>> ScanPoints;
#ifdef USE_VECTOR
    std::vector<std::vector<OnePointInfo>> EchoPoints;
#else
	OnePointInfo nonePointInfo;
	size_t** EchoPointsIndex = new size_t*[2];
	OnePointInfo** EchoPoints = new OnePointInfo*[2];
	PointSizeCn EchoPointsSize;
#endif // USE_VECTOR



public:
    CustomerAnalysisBase();
    ~CustomerAnalysisBase();
    /// <summary>
    /// 
    /// </summary>
    /// <param name="line_no">0-191</param>
    /// <param name="line_id">0-959</param>
    /// <returns></returns>
    static inline size_t create_id(size_t line_no, size_t line_id)
    {
        return line_no * ONELINE_POINTNUM + line_id;
    }
    void analysis(std::vector<uint8>& buf, std::vector<base::TimooPoint>& block_points, bool& is_last_packet);
#ifdef USE_VECTOR
    virtual void getPoints(std::vector<uint8>& buf, std::vector<std::vector<OnePointInfo>>& out_Points) = 0;
#else
    virtual void getPoints(std::vector<uint8>& buf, OnePointInfo**& out_Points, PointSizeCn& out_size, 
					        std::vector<base::TimooPoint>& block_points) = 0;
    /// <summary>
    /// 
    /// </summary>
    /// <param name="echo">0-第一重 1-第二重</param>
    /// <param name="line">0-191</param>
    /// <param name="line_id">0-959 </param>
    /// <param name="srcHAngle">0 15 30 45 </param>
    /// <param name="old"></param>
    /// <returns></returns>
    OnePointInfo& create_unsafe(int echo,int line_no, size_t line_id, int srcHAngle, bool& old)
    {
        size_t id = create_id(line_no, line_id);
        if (EchoPointsIndex[echo][line_no * HANGLE_MAX + srcHAngle] == -1)
        {
            EchoPointsIndex[echo][line_no * HANGLE_MAX + srcHAngle] = id;
            old = false;
            return EchoPoints[echo][id];
        }
        else
        {
            old = true;
            return EchoPoints[echo][id];
        }
    }
    void clear()
    {
#ifdef USE_VECTOR

        EchoPoints[0].clear();
        EchoPoints[1].clear();
#else
        memset(EchoPoints[0], 0, LIDAR_LINE_MAX * ONELINE_POINTNUM * sizeof(OnePointInfo));
        memset(EchoPoints[1], 0, LIDAR_LINE_MAX * ONELINE_POINTNUM * sizeof(OnePointInfo));
        std::fill(EchoPointsIndex[0], EchoPointsIndex[0] + LIDAR_LINE_MAX * HANGLE_MAX, -1);
        EchoPointsSize.firstSize = 0;
        EchoPointsSize.LastSize = 0;
#endif // USE_VECTOR
    }

    /// <summary>
    /// 获取一个点
    /// </summary>
    /// <param name="EchoType">1-第一重 2-第二重</param>
    /// <param name="LineNo">1-192</param>
    /// <param name="HAngle">0 15 30 45 ....</param>
    /// <returns></returns>
    inline OnePointInfo& getData(int EchoType, unsigned int LineNo, unsigned int HAngle)
    {
        assert(EchoType < 3);
        assert(LineNo - 1 < LIDAR_LINE_MAX);
        assert(HAngle <= HANGLE_MAX);
        assert(EchoType <= 2);

        int index = EchoPointsIndex[EchoType - 1][(LineNo - 1) * HANGLE_MAX + HAngle];
        if (index >= 0)
            return EchoPoints[EchoType - 1][index];
        else
            return nonePointInfo;
    }

#endif
};

} // namespace driver
} // namespace timoo

