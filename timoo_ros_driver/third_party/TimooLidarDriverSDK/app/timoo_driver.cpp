
#include "timoo_driver.h"

#include "common/common.h"
#include "input/input_factory.h"
#include "data_parser/parser_factory.h"
#include "interface/base_data.h"
#include <cstddef>
#include <string>
#include <cstring>      // for memset
#include <unistd.h>     // for close()
#include <sys/socket.h> // for socket()
#include <netinet/in.h> // for sockaddr_in
#include <arpa/inet.h>  // for inet_addr()

namespace timoo {
namespace driver {

#define GPS_PACKAGE_SIZE 512

// Timoo sensor buffer size
static std::map<base::SensorType, size_t> SENSOR_BUFFER_SIZE{
    {base::SensorType::TIMOO16, 1206},
    {base::SensorType::TIMOO32, 1206},
    {base::SensorType::TIMOO1550STD, 1351},
    {base::SensorType::TIMOO1550, 1280},
    {base::SensorType::TIMOO128, 1238},
    {base::SensorType::TIMOO128V, 841},
};

// Timoo sensor status packet buffer size
static std::map<base::SensorType, size_t> SENSOR_STATUS_BUFFER_SIZE{
    {base::SensorType::TIMOO16, 1206},
    {base::SensorType::TIMOO32, 1206},
    {base::SensorType::TIMOO1550STD, 1351}, // TODO: temporarily add
    {base::SensorType::TIMOO1550, 1280},
    {base::SensorType::TIMOO128, 1238},
    {base::SensorType::TIMOO128V, 841},
};

// Timoo sensor imu packet buffer size
static std::map<base::SensorType, size_t> SENSOR_IMU_BUFFER_SIZE{
    {base::SensorType::TIMOO16, 60},
};


TimooDriver::TimooDriver() {
    for(int i = 0 ;i < 32000 ; i++){
        base::TimooPoint point;
        point.x = nanf("");
        point.y = nanf("");
        point.z = nanf("");
        point.intensity = nanf("");  // Use 0 to represent invalid or undefined intensity;
        point.ring_id = nanf("");
        point.time = nanf("");
        ZERO_BLOCK_POINTS.push_back(point);
    }

}


TimooDriver::~TimooDriver() {
    // Stop running
    Stop();
}
void TimooDriver::afterReadEnd()
{
    std::cout << "[TimooDriver] Reading End." << std::endl;
}

void TimooDriver::afterReadFail()
{
    std::cout << "[TimooDriver] Reading Fail." << std::endl;
}


bool TimooDriver::Init(base::DeviceConfig config) {

    if (config.file_name.empty()) {
        input_ = InputFactory::MakeInputUDP(config.host_ip,config.udp_port, SENSOR_BUFFER_SIZE[config.lidar_type]);
        status_input_ = InputFactory::MakeInputUDP(config.host_ip,config.status_port, SENSOR_STATUS_BUFFER_SIZE[config.lidar_type]);
        imu_input_ = InputFactory::MakeInputUDP(config.host_ip,config.imu_dst_port, SENSOR_IMU_BUFFER_SIZE[config.lidar_type]);
    } else {
        input_ = InputFactory::MakeInputPCAP(config.file_name, 0.1);
    }

    parser_ = ParserFactory::MakeDataParser(config.lidar_type);
    parser_->Init(config);

    // Initialize AdvancedSetting
    adv_func_ = std::make_shared<AdvancedSetting>(config.status_port);

    running_is_stop_ = false;

    cfg_ = config;

    return true;
}

void TimooDriver::ImuDataGettingFunc()
{
    while(!running_is_stop_)
    {
        if (imu_input_ == nullptr) { break; }

        RawDataPtr data(new RawData(DataType::IMU));
        int res = imu_input_->GetPackage(data.get());

        // Check result
        if (res == -2) // there are no more packets to read
        {
            afterReadEnd();
            return;
        }
        else if (res == -1) // an error occurred
        {
            afterReadFail();
            return;
        }
        else if (res == -3) // an timeout occurred
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.001));
            continue;
        }

        // Set raw data to parser
        parser_mutex_.lock();
        parser_->SetRawData(data);
        parser_mutex_.unlock();
    }

}

void TimooDriver::statusDataGettingFunc() 
{
    while (!running_is_stop_)
    {
        if (status_input_ == nullptr) { break; }

        // Read data
        RawDataPtr data(new RawData(DataType::STATUS));
        int res = status_input_->GetPackage(data.get());

        // For lidar setting, TM16 only
        adv_func_->updateBuffer(data->d);

        // Check result
        if (res == -2) // there are no more packets to read
        {
            afterReadEnd();
            return;
        }
        else if (res == -1) // an error occurred
        {
            afterReadFail();
            return;
        }
        else if (res == -3) // an timeout occurred
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.001));
            continue;
        }

        // Set raw data to parser
        parser_mutex_.lock(); 
        parser_->SetRawData(data);
        parser_mutex_.unlock();

        std::this_thread::sleep_for(std::chrono::duration<double>(0.001));
    }
}

void TimooDriver::dataGettingFunc()
{

    // Get one package
    while (!running_is_stop_)
    {
        if (input_ == nullptr) { break; }
        
        // Read data
        RawDataPtr data(new RawData);
        int res = input_->GetPackage(data.get());

        // Check result
        if (res == -2) // there are no more packets to read
        {
            afterReadEnd();
            return;
        }
        else if (res == -1) // an error occurred
        {
            afterReadFail();
            return;
        }
        else if (res == -3) // an timeout occurred
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.001));
            continue;
        }

        // Parse data
        if (data->size == GPS_PACKAGE_SIZE) {
            // TODO: GPS data or other data type
        } else {
            // Set raw data to parser
            parser_mutex_.lock();
            parser_->SetRawData(data);
            parser_mutex_.unlock();
        }

    }
}

void TimooDriver::correctPktTimestamp(base::TimooPacketDataPtr& pkt_data) {

    // If using tail time, reassign the value
    if (cfg_.use_tail_time) { 
        points_buffer_->timestamp = points_buffer_->points.back().time; 
    } else {
        points_buffer_->timestamp = points_buffer_->points.front().time; 
    }

    for (auto & p : points_buffer_->points) {
        p.time = p.time - points_buffer_->timestamp;
    }

}

void TimooDriver::collectPoints(base::TimooPacketDataPtr& pkt_data)
{
    if (pkt_data == nullptr || pkt_data->points.empty()) { return; }

    if (points_buffer_ == nullptr) {
        points_buffer_ = std::make_shared<base::TimooPointCloud>();
        points_buffer_->timestamp = pkt_data->timestamp;
    }

    if (pkt_data->is_last_packet) {

        points_buffer_->points.insert(points_buffer_->points.end(), 
                                      pkt_data->points.begin(), pkt_data->points.begin() + pkt_data->cut_point_index);

        // Correct points time
        correctPktTimestamp(pkt_data);

        // Publish full frame points
        if (cfg_.fixed_points_count) {
            if(points_buffer_->points.size() > 32000) {
                points_buffer_->points.erase(points_buffer_->points.begin() + 32000 , points_buffer_->points.end());
            }

            if(points_buffer_->points.size() < 32000) {
                size_t len = 32000 - points_buffer_->points.size(); 
                points_buffer_->points.insert(points_buffer_->points.end(), ZERO_BLOCK_POINTS.begin(),ZERO_BLOCK_POINTS.begin() + len);
            }
        }

        if (points_buffer_->points.size() > 10) {
            frame_cloud_callback_(points_buffer_);
        }

        // Restart insert next frame points to points_buffer_
        points_buffer_->points.clear();

        points_buffer_->timestamp = pkt_data->cut_point_time;
        points_buffer_->points.insert(points_buffer_->points.end(), 
                                      pkt_data->points.begin() + pkt_data->cut_point_index, 
                                      pkt_data->points.end());
    } else {

        points_buffer_->points.insert(points_buffer_->points.end(), pkt_data->points.begin(), pkt_data->points.end());
    }
}

void TimooDriver::collectImuData(base::TimooPacketDataPtr& pkt_data)
{
    imudata_buffer_ = std::make_shared<base::IMUData>();
    imudata_buffer_->time = pkt_data->imu_data.time;
    imudata_buffer_->Accel_x = pkt_data->imu_data.Accel_x;
    imudata_buffer_->Accel_y = pkt_data->imu_data.Accel_y;
    imudata_buffer_->Accel_z = pkt_data->imu_data.Accel_z;
    imudata_buffer_->Gyro_x = pkt_data->imu_data.Gyro_x;
    imudata_buffer_->Gyro_y = pkt_data->imu_data.Gyro_y;
    imudata_buffer_->Gyro_z = pkt_data->imu_data.Gyro_z;
    imudata_buffer_->temperature = pkt_data->imu_data.temperature;
    frame_imu_callback_(imudata_buffer_);
}

void TimooDriver::dataHandlingFunc()
{
    // Get one package
    while (!running_is_stop_)
    {
        base::TimooPacketDataPtr timoo_pkt_data;
        int res = parser_->ParsingRawData(timoo_pkt_data);

        // Rawdata buffer is empty, waiting a little bit moment
        if (res == -1) {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.005));
        }

        // after handle data
        if (timoo_pkt_data == nullptr) { continue; }

        // Collect data
        if(timoo_pkt_data->is_imu) {
            collectImuData(timoo_pkt_data);
        } else {
            collectPoints(timoo_pkt_data); 
        }
    }
}

void TimooDriver::Start()
{
    running_is_stop_ = false;

    if (data_handle_thread_ == nullptr)
    {
        data_handle_thread_ = std::make_shared<std::thread>([this]()
        {
            this->dataHandlingFunc();
        });
    }

    if (data_get_thread_ == nullptr)
    {
        data_get_thread_ = std::make_shared<std::thread>([this]()
        {
            this->dataGettingFunc();
        });
    }

    if (status_data_get_thread_ == nullptr)
    {
        status_data_get_thread_ = std::make_shared<std::thread>([this]()
        {
            this->statusDataGettingFunc();
        });
    }
    
    if (cfg_.use_imu_data && imu_data_thread_ == nullptr)
    {
        imu_data_thread_ = std::make_shared<std::thread>([this]()
         {   
            this->ImuDataGettingFunc();
         });
    }
}

void TimooDriver::Stop()
{

    std::cout << "[TimooDriver] driver stop " << std::endl;
    running_is_stop_ = true;

    if (data_handle_thread_ != nullptr)
    {
        data_handle_thread_->join();
        data_handle_thread_ = nullptr;
    }

    if (data_get_thread_ != nullptr)
    {
        data_get_thread_->join();
        data_get_thread_ = nullptr;
    }

    if (status_data_get_thread_ != nullptr)
    {
        status_data_get_thread_->join();
        status_data_get_thread_ = nullptr;
    }

    if (imu_data_thread_ != nullptr)
    {
        imu_data_thread_->join();
        imu_data_thread_ = nullptr;
    }
}

} // namespace driver
} // namespace timoo
