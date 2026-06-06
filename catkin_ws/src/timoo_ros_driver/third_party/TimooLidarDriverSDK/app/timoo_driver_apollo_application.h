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

namespace timoo {
namespace driver {

class TimooDriverApolloApplication
{

    typedef std::function<void(base::TimooPointCloudPtr)> FrameCloudCallBack;
    typedef std::function<void(base::TimooPointScanPtr)>  FrameScanCallBack;

public:

    explicit TimooDriverApolloApplication();
    ~TimooDriverApolloApplication();

    bool Init(base::DeviceConfig config);

    void lidarDataSettingFunc(RawDataPtr& data);
    void statusDataSettingFunc(RawDataPtr& data);

    void Start();
    void Stop();

private:
    // Running func - Thread for data handling
    void dataHandlingFunc();
    
    void afterReadEnd();
    void afterReadFail();

    void collectPoints(base::TimooPacketDataPtr& pkt_data);

    // Data parser interface
    BaseParserPtr parser_;
    std::mutex parser_mutex_;

    base::TimooPointCloudPtr points_buffer_;

    FrameCloudCallBack frame_cloud_callback_;

    // Config params for Timoo sensor
    base::DeviceConfig cfg_;

    // Thread tools
    std::shared_ptr<std::thread> data_handle_thread_ = nullptr;
    // std::shared_ptr<std::thread> data_get_thread_ = nullptr;
    // std::shared_ptr<std::thread> status_data_get_thread_ = nullptr;
    std::atomic<bool> running_is_stop_;

public:
    void registerFrameCloudCallBack(std::function<void(base::TimooPointCloudPtr)> func) {
        frame_cloud_callback_ = std::move(func);
    }

};


} // namespace driver
} // namespace timoo
