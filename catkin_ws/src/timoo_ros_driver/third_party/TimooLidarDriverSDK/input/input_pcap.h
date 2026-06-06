#pragma once
#include <pcap.h>

#include "interface/base_input.h"


namespace timoo {
namespace driver {



class InputPCAP : public BaseInput
{
public:
    explicit InputPCAP();

    bool Init(const std::string& file_path, const double time_period);

    int GetPackage(RawData* data) override;

private:
    // Params
    double time_period_ = 0;

    // PCAP tools
    pcap_t *pcap_;

    // Data pointer
    const u_char* data_;
    
};

} // namespace driver
} // namespace timoo
