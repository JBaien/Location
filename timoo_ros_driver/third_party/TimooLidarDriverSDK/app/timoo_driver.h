#pragma once
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <atomic>
#include <vector>
#include <functional>

#include "common/device_config.h"
#include "common/point_type.h"
#include "interface/base_input.h"
#include "interface/base_parser.h"
#include "func/advanced_setting.h"

namespace timoo {
namespace driver {

class TimooDriver
{

    typedef std::function<void(base::TimooPointCloudPtr)> FrameCloudCallBack;
    typedef std::function<void(base::TimooPointScanPtr)>  FrameScanCallBack;
    typedef std::function<void(base::TimooIMUPtr)>  FrameImuCallBack;


public:

    explicit TimooDriver();
    ~TimooDriver();

    bool Init(base::DeviceConfig config);

    // void lidarstart(int);
    // void lidarsleep(int);
    void Start();
    void Stop();
    bool send_ = true;
    int speed_;
    int status_;
private:
    // Running func - Thread for data handling
    void dataHandlingFunc();
    // Running func - Thread for data getting
    void dataGettingFunc();

    // Running func - Thread for status getting
    void statusDataGettingFunc();

    void ImuDataGettingFunc();
    
    void afterReadEnd();
    void afterReadFail();

    void collectPoints(base::TimooPacketDataPtr& pkt_data);

    void collectImuData(base::TimooPacketDataPtr& pkt_data);

    void correctPktTimestamp(base::TimooPacketDataPtr& pkt_data);
    
    // Input interface
    BaseInputPtr input_;
    BaseInputPtr status_input_;
    BaseInputPtr imu_input_;

    // Data parser interface
    BaseParserPtr parser_;
    std::mutex parser_mutex_;

    base::TimooPointCloudPtr points_buffer_;

    FrameCloudCallBack frame_cloud_callback_;

    base::TimooIMUPtr imudata_buffer_;
    FrameImuCallBack frame_imu_callback_;

    // Config params for Timoo sensor
    base::DeviceConfig cfg_;

    // Thread tools
    std::shared_ptr<std::thread> data_handle_thread_ = nullptr;
    std::shared_ptr<std::thread> data_get_thread_ = nullptr;
    std::shared_ptr<std::thread> status_data_get_thread_ = nullptr;
    std::shared_ptr<std::thread> imu_data_thread_ = nullptr;
    std::atomic<bool> running_is_stop_;

    std::vector<u_char > buf_;
    std::mutex buf_mutex_;

    AdvancedSettingPtr adv_func_;

    std::vector<base::TimooPoint> ZERO_BLOCK_POINTS;

public:
    void registerFrameCloudCallBack(std::function<void(base::TimooPointCloudPtr)> func) {
        frame_cloud_callback_ = std::move(func);
    }

    void registerFrameImuCallBack(std::function<void(base::TimooIMUPtr)> func) {
        frame_imu_callback_ = std::move(func);
    }

    AdvancedSettingPtr getAdvFunc() { return adv_func_; };

};


} // namespace driver
} // namespace timoo
