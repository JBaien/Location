
#include "tm128_parser.h"

#include <cmath>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string.h>
#include <unistd.h>

namespace timoo {
namespace driver {

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


int TM128DataParser::Init(const base::DeviceConfig& config) {

    cfg_ = config;

 	lidardata.azimuth.resize(128);
	lidardata.distance.resize(128);
	lidardata.intensity.resize(128);
	lidardata.mtimestamp.resize(128);
	lidardata.Row_id.resize(128);
	lidardata.azimuth_diff.resize(128);

    std::cout << "Init!" << std::endl;
    //isStart_ = false;
    // if (cfg_.cut_angle > 0.0) {
    //     cfg_.cut_angle = int((360.0 + cfg_.cut_angle) * 100) % 36000;
    // }

    // if (config.calibration_file.empty()) {
    //     calibration_.read("/opt/timoo_lidar_driver/params/tm16.yaml");
    // } else {
    //     calibration_.read(config.calibration_file);
    // }

    // // Set up cached values for sin and cos of all the possible headings
    // for (size_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) 
    // {
    //     float rotation = (ROTATION_RESOLUTION * rot_index) * M_PI / 180.0;
    //     cos_rot_table_[rot_index] = cosf(rotation);
    //     sin_rot_table_[rot_index] = sinf(rotation);
    // }

    // // For points level time calculation
    // pkt_duration_ = SINGLE_FIRING * 32.0 * 10.0;
    // block_duration_ = SINGLE_FIRING * 16.0;
    // single_point_duration_ = SINGLE_FIRING;

    return 0;
}


void TM128DataParser::setFrameStartInfo(const double& timestamp)
{
    pkt_data_ = std::make_shared<base::TimooPacketData>();
    pkt_data_->points.reserve(33800);
    pkt_data_->is_last_packet = false;
    pkt_data_->timestamp = timestamp;
}

void TM128DataParser::setFrameEndInfo(const double& timestamp) 
{
    // if (pkt_data_->points.empty()) { return; }
    pkt_data_->is_last_packet = true;
    pkt_data_->cut_point_index = pkt_data_->points.size();    
    pkt_data_->cut_point_time = timestamp;
}

void TM128DataParser::correctTimestamp(const base::Time& cur_pkt_time) 
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

int TM128DataParser::packetDataParse(const RawDataPtr raw_data)
{

    float mdiff;
	double angle;

    TM128_MSOP *msop = new TM128_MSOP; //struct TM128

    memcpy(msop, raw_data->d.data(), sizeof(TM128_MSOP));

    setFrameStartInfo(msop->Timestamp.timestamp + (0)*DSR_TOFFSET - BAG_TOFFSET);



    for (int i = 0; i < 12; i++){
        if (i < 11) {
            mdiff = (msop->datablock[i + 1].Azimuth - msop->datablock[i].Azimuth) / 100.f;
            mdiff = mdiff > 0 ? mdiff : mdiff + 360;
        }

        for (int j = 0; j < 32; j++){
        
            lidardata.distance[ChannelToAngleNoTM128[j]].push_back((msop->datablock[i].channeldata[j].distance));
            lidardata.intensity[ChannelToAngleNoTM128[j]].push_back(msop->datablock[i].channeldata[j].reflectivity);
            lidardata.mtimestamp[ChannelToAngleNoTM128[j]].push_back(msop->Timestamp.timestamp + (i * 32 + j + 1)*DSR_TOFFSET - BAG_TOFFSET);

            angle = (msop->datablock[i].Azimuth) / 100.f/* + (mdiff/32) * j*/;

            angle = angle < 360 ? angle : angle - 360;


            lidardata.Row_id[ChannelToAngleNoTM128[j]].push_back(msop->datablock[i].Row_id);
            lidardata.azimuth[ChannelToAngleNoTM128[j]].push_back(angle);
            lidardata.azimuth_diff[ChannelToAngleNoTM128[j]].push_back(msop->datablock[i].Azimuth_diff);

        }
    }

    if (last_angle_ <= 180 && angle >= 180/*(abs(angle - 180)<0.4 && lidardata.distance[0].size() > 100)*/ || lidardata.distance[0].size()>10000){

        sendpoint(lidardata);
        lidardata.azimuth.clear();
        lidardata.distance.clear();
        lidardata.intensity.clear();
        lidardata.mtimestamp.clear();
        lidardata.Row_id.clear();
        lidardata.azimuth_diff.clear();
        lidardata.azimuth.resize(128);
        lidardata.distance.resize(128);
        lidardata.intensity.resize(128);
        lidardata.mtimestamp.resize(128);
        lidardata.Row_id.resize(128);
        lidardata.azimuth_diff.resize(128);
    }
    
    last_angle_ = angle;
				

    // std::cout << "2" << std::endl;
    delete msop;

    

    return 0;
}

void TM128DataParser::sendpoint(LidarData lidardata)
{

    double time ;
   for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 32; j++){
            //float theta = VerticalDefaultValueTM128[i * 32 + j];
            float theta = AngleNoToAngleValuetm128[i][j];
            cosTheta[i][j] = cos(theta * M_PI / 180);
            sinTheta[i][j] = sin(theta * M_PI / 180);
            //std::cout << cosTheta[i][j] << "    " <<sinTheta[i][j] << std::endl;
        }
    }

	for (int i = 0; i < 32; i++){
			int width = (int)(lidardata.distance[i].size());
			for (int j = 0; j < width; j++){

				if (lidardata.distance[i][j] <= 0)//*|| (lidardata.distance[i][j] / 100.f > 0 && lidardata.distance[i][j] / 100.f<40 && lidardata.intensity[i][j]<15)*/)//滤除距离为0
					continue;

				//声明输出点云类型
				int index = -1;

				float AzimuthDiff = 0;
				if (i % 2 != 0){ //基数左右
					AzimuthDiff = azimuthDiff_;
					/*	printf("AzimuthCalib[0]  %d \n", AzimuthCalib[0]);*/
				}

				if (lidardata.azimuth[i][j] > 15  + AzimuthDiff && lidardata.azimuth[i][j] < 75 - AzimuthDiff){
					index = 2;
					lidardata.azimuth[i][j] = (lidardata.azimuth[i][j] - 15 - AzimuthDiff) * 2 - 60;
				}
				else if (lidardata.azimuth[i][j] >105  + AzimuthDiff && lidardata.azimuth[i][j] < 165  - AzimuthDiff){
					index = 0;
					lidardata.azimuth[i][j] = (lidardata.azimuth[i][j] - 105 - AzimuthDiff) * 2 - 60;
				}
				else if (lidardata.azimuth[i][j] >195 + AzimuthDiff && lidardata.azimuth[i][j] < 255 - AzimuthDiff){
					index = 3;
					lidardata.azimuth[i][j] = (lidardata.azimuth[i][j] - 195 - AzimuthDiff) * 2 - 60;
				}
				else if (lidardata.azimuth[i][j] >285 + AzimuthDiff && lidardata.azimuth[i][j] < 345 - AzimuthDiff){
					index = 1;
					lidardata.azimuth[i][j] = (lidardata.azimuth[i][j] - 285 - AzimuthDiff) * 2 - 60;
				}

				lidardata.azimuth[i][j] = lidardata.azimuth[i][j]>360 ? lidardata.azimuth[i][j] - 360 : lidardata.azimuth[i][j];



				if (index == -1)
					continue;

				lidardata.azimuth[i][j] = lidardata.azimuth[i][j] - plane4Azimuth_[index];


				//pcl::PointXYZRGB PointTemp;
                base::TimooPoint PointTemp;
				//添加x,y,z坐标   单位为cm，所以要除以100
				PointTemp.y = -(lidardata.distance[i][j] / 200.f) * cosTheta[index][i] * sin(lidardata.azimuth[i][j] * M_PI / 180);
				PointTemp.x = (lidardata.distance[i][j] / 200.f) * cosTheta[index][i] * cos(lidardata.azimuth[i][j] * M_PI / 180);
				PointTemp.z = (lidardata.distance[i][j] / 200.f) * sinTheta[index][i];
                PointTemp.intensity = lidardata.intensity[i][j];

				//根据反射强度显示颜色---修改版本
				if (lidardata.intensity[i][j] > 255){
					lidardata.intensity[i][j] = 255;
				}


				if (lidardata.distance[i][j] / 200.f > 1 /*&& lidardata.distance[i][j]/200.f < 150*/){
                    pkt_data_->points.emplace_back(PointTemp);
                    time = lidardata.mtimestamp[i][j];
				}
			}
		}

        setFrameEndInfo(time);


}

int TM128DataParser::statusDataParse(const RawDataPtr raw_data)
{
    //std::cout << "1"<< std::endl;
    
    TM_128_DIFOP difop;
    memcpy(&difop,raw_data->d.data(),sizeof(TM_128_DIFOP));
   // std::cout << "2"<< std::endl;

	float vertical128[32];
	for (int i = 0; i < 32; i++){
		vertical128[i] = VerticalDefValueTM128[i];
				//printf("vertical128 = %f\n", vertical128[i]);
		}

		for (int i = 0; i < 32; i++){
			AngleNoToAngleValuetm128[0][i] = vertical128[i];
			for (int j = 1; j < 4; j++){
				AngleNoToAngleValuetm128[j][i] = vertical128[i] - 0.2*j;
			}
		}

		//两板之间的水平角度
		//AzimuthCalib[0] = difop->BoardAzimuth;
		azimuthDiff_ = difop.BoardAzimuth/100.f;
        //std::cout << "azimuthDiff_" << azimuthDiff_ << std::endl;
		for (int i = 0; i < 4; i++){
			plane4Azimuth_[i] =difop.Plane4Azimuth[i]/100.f;
            //std::cout <<"plane4Azimuth_[" << i << "]" <<plane4Azimuth_[i]<<std::endl;
		}
 
    vertical_calibrate_updated_ = true;
    return 0;
}

int TM128DataParser::parse(const RawDataPtr raw_data) {
    if (raw_data->type == DataType::STATUS) {
        return statusDataParse(raw_data);
    } else if (raw_data->type == DataType::PACKET) {
        
        if (!vertical_calibrate_updated_) {
            std::cout << "[TM128DataParser] vertical angle calibration not update, please check status packet!" << std::endl;
        }

        return packetDataParse(raw_data);
    } else {
        return -1;
    }

    return 0;
}



} // namespace driver
} // namespace timoo

