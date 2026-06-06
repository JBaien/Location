#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <mutex>

#include "base_data.h"
#include "common/point_type.h"
#include "common/device_config.h"

namespace timoo {
namespace driver {



/// ////////////////////////////
/// BaseParser
/// ////////////////////////////

class BaseParser
{
public:
    BaseParser() = default;
    virtual ~BaseParser() = default;

    virtual int Init(const base::DeviceConfig& config) = 0;

    int SetRawData(const RawDataPtr& data) {
        std::lock_guard<std::mutex> lock(rawdata_list_mutex_);
        rawdata_list_.push_back(data);
        return 0;
    };

    int ParsingRawData(base::TimooPacketDataPtr& data_out) {

        RawDataPtr rawdata;
        // Extract raw data from list
        rawdata_list_mutex_.lock();
        if (rawdata_list_.empty()) {
            rawdata_list_mutex_.unlock();
            return -1;
        }
        rawdata = rawdata_list_.front();
        rawdata_list_.pop_front();
        rawdata_list_mutex_.unlock();

        // Parse raw data
        int parse_res = parse(rawdata);
        if (parse_res) { return parse_res; }

        data_out = pkt_data_;
        pkt_data_ = nullptr;
        return 0;
    };

    virtual int parse(const RawDataPtr raw_data) = 0;

protected:

    base::DeviceConfig cfg_;

    base::TimooPacketDataPtr pkt_data_;

    // bool is_data_ready_ = false;

    std::list<RawDataPtr> rawdata_list_;
    std::mutex rawdata_list_mutex_;
};

typedef std::shared_ptr<BaseParser> BaseParserPtr;

} // namespace driver
} // namespace timoo
