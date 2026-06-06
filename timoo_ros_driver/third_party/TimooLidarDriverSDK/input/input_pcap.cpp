#include "input_pcap.h"

#include <iostream>
#include <chrono>
#include <thread>

namespace timoo {
namespace driver {


InputPCAP::InputPCAP()
{

}

bool InputPCAP::Init(const std::string& file_path, const double time_period)
{
    time_period_ = time_period;

    char pcap_error_buffer_[PCAP_ERRBUF_SIZE];
    pcap_open_offline(file_path.c_str(), pcap_error_buffer_);
    
    // Open pcap file
    if ((pcap_ = pcap_open_offline(file_path.c_str(), pcap_error_buffer_)) == NULL)
    {
        std::cout << "[InputPCAP::Init] read pcap file wrong!" << std::endl;
        return false;
    }
    else
    {
        std::cout << "[InputPCAP::Init] Begin to reading pcap file successful!" << std::endl;
    }

    return true;
}

int InputPCAP::GetPackage(RawData* data) 
{   
    // std::cout << " enter GetPackage "<< std::endl;
    // Read data
    struct pcap_pkthdr* header;
    int res = pcap_next_ex(pcap_, &header, &data_);

    // std::cout << res<< std::endl;

    // Check result
    if (res == -2) // there are no more packets to read from the savefile, close and reopen file
    {
        std::cout << "[InputPCAP::GetPackage] PCAP file reading end." << std::endl;
        return -2;
    }
    else if (res == -1) // an error occurred while reading the packet
    {
        std::cout << "[InputPCAP::GetPackage] PCAP file reading error!" << std::endl;
        return -1;
    }
    else
    {

        unsigned int time_sec = header->ts.tv_sec;
        unsigned int time_usec = header->ts.tv_usec;

        data->size = header->caplen - ETHERNET_HEADER_SIZE;
        data->d.resize(data->size);
        data->d.assign(data_, data_ + data->size);

        // data->time = ros::Time(time_sec, time_usec * 1000);
    }  

    // Periodic sleep
    if (time_period_ > 0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(time_period_));
    }

    return 0; 
}

} // namespace driver
} // namespace timoo
