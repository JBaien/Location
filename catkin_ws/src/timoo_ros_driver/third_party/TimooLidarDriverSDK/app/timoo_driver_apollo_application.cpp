
#include "timoo_driver_apollo_application.h"

#include "common/common.h"
#include "data_parser/parser_factory.h"
#include "interface/base_data.h"
#include <cstddef>
#include <string>


namespace timoo {
namespace driver {

#define GPS_PACKAGE_SIZE 512

// Timoo sensor buffer size
static std::map<base::SensorType, size_t> SENSOR_BUFFER_SIZE{
    {base::SensorType::TIMOO16, 1206},
    {base::SensorType::TIMOO32, 1206},
    {base::SensorType::TIMOO1550STD, 1351},
    {base::SensorType::TIMOO1550, 1280},
};

// Timoo sensor status packet buffer size
static std::map<base::SensorType, size_t> SENSOR_STATUS_BUFFER_SIZE{
    {base::SensorType::TIMOO16, 1206},
    {base::SensorType::TIMOO32, 1206},
    {base::SensorType::TIMOO1550STD, 1351}, // TODO: temporarily add
    {base::SensorType::TIMOO1550, 1280},
};


TimooDriverApolloApplication::TimooDriverApolloApplication() {}


TimooDriverApolloApplication::~TimooDriverApolloApplication() {
    // Stop running
    Stop();
}
void TimooDriverApolloApplication::afterReadEnd()
{
    std::cout << "[TimooDriverApolloApplication] Reading End." << std::endl;
}

void TimooDriverApolloApplication::afterReadFail()
{
    std::cout << "[TimooDriverApolloApplication] Reading Fail." << std::endl;
}


bool TimooDriverApolloApplication::Init(base::DeviceConfig config) {

    parser_ = ParserFactory::MakeDataParser(config.lidar_type);
    parser_->Init(config);

    running_is_stop_ = false;

    cfg_ = config;

    return true;
}


void TimooDriverApolloApplication::statusDataSettingFunc(RawDataPtr& data)
{
 
    // Set raw data to parser
    parser_mutex_.lock();
    parser_->SetRawData(data);
    parser_mutex_.unlock();
    data = nullptr;

}

void TimooDriverApolloApplication::lidarDataSettingFunc(RawDataPtr& data)
{

    // Parse data
    if (data->size == GPS_PACKAGE_SIZE) {
        // TODO: GPS data or other data type
    } else {
        // Set raw data to parser
        parser_mutex_.lock();
        parser_->SetRawData(data);
        parser_mutex_.unlock();
        data = nullptr;
    }

}

void TimooDriverApolloApplication::collectPoints(base::TimooPacketDataPtr& pkt_data)
{
    if (pkt_data == nullptr || pkt_data->points.empty()) { return; }

    if (points_buffer_ == nullptr) {
        points_buffer_ = std::make_shared<base::TimooPointCloud>();
        points_buffer_->timestamp = pkt_data->timestamp;
    }


    if (pkt_data->is_last_packet) {

        points_buffer_->points.insert(points_buffer_->points.end(), 
                                      pkt_data->points.begin(), pkt_data->points.begin() + pkt_data->cut_point_index);

        // If using tail time, reassign the value
        if (cfg_.use_tail_time) { points_buffer_->timestamp = pkt_data->cut_point_time; }

        // Publish full frame points
        frame_cloud_callback_(points_buffer_);

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


void TimooDriverApolloApplication::dataHandlingFunc()
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
        if (timoo_pkt_data) { collectPoints(timoo_pkt_data); }
    }
}

void TimooDriverApolloApplication::Start()
{
    running_is_stop_ = false;

    if (data_handle_thread_ == nullptr)
    {
        data_handle_thread_ = std::make_shared<std::thread>([this]()
        {
            this->dataHandlingFunc();
        });
    }

}

void TimooDriverApolloApplication::Stop()
{

    std::cout << "driver stop " << std::endl;
    running_is_stop_ = true;

    if (data_handle_thread_ != nullptr)
    {
        data_handle_thread_->join();
        data_handle_thread_ = nullptr;
    }
}

} // namespace driver
} // namespace timoo
