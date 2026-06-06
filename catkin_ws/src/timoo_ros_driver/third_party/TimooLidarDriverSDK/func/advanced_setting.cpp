#include "advanced_setting.h"



namespace timoo {
namespace driver {

AdvancedSetting :: AdvancedSetting(int port ){

    port_ = port;
}

AdvancedSetting::~AdvancedSetting(){}

bool AdvancedSetting::checkpacket(const std::vector<u_char>& data)
{
    if((data[962]!=first_packet_buf_[962]) && (data[963]!=first_packet_buf_[963]))
    {
        return false;
    }

    for(int i = 834 ;i < 898 ;i ++)
    {
        if(data[i] != first_packet_buf_[i])
        {
            return false;
        }
    }



    return true;
}

void AdvancedSetting::updateBuffer(const std::vector<u_char>& data){
    // 加锁保护 buf_ 更新
    std::lock_guard<std::mutex> lock(buf_mutex_);

    buf_ = data;

    //std::cout << "recv buf_" << std::endl;
    if(first_packet_buf_.empty())
    {
        first_packet_buf_ = data;
    }
}

bool AdvancedSetting::setDormant(){

    if(buf_.empty()){return false;}


   if(!checkpacket(buf_))
    {
        SendFirstPacket();
        std::cout << "error buf_" << std::endl;
        return false;
    }

    unsigned char UCWP_buff[1206];

    // 帧头识别符：0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA
    UCWP_buff[0] = 0xAA;
    UCWP_buff[1] = 0x00;
    UCWP_buff[2] = 0xFF;
    UCWP_buff[3] = 0x11;
    UCWP_buff[4] = 0x22;
    UCWP_buff[5] = 0x22;
    UCWP_buff[6] = 0xAA;
    UCWP_buff[7] = 0xAA;

    // 拷贝buf_内容（加锁保护）
    {
    std::lock_guard<std::mutex> lock(buf_mutex_);

        for (int i = 8; i < 46; ++i) {
            UCWP_buff[i] = buf_[i];
        }

        UCWP_buff[46] = 0x00;
        UCWP_buff[47] = 0x01;

        for (int i = 48; i < 1204; ++i) {
            UCWP_buff[i] = buf_[i];
        }
    }

    // 帧尾
    UCWP_buff[1204] = 0x0F;
    UCWP_buff[1205] = 0xF0;

    std::string ip_addr_ = std::to_string(UCWP_buff[10]) + "." +
                           std::to_string(UCWP_buff[11]) + "." + 
                           std::to_string(UCWP_buff[12]) + "." + 
                           std::to_string(UCWP_buff[13]);

    //std::cout << ip_addr_ <<"    " <<   port_ <<" sleep!!!!" <<std::endl;

    int g_socketudp = socket(2,2,0);
    sockaddr_in toaddr;
    toaddr.sin_family = AF_INET;
    toaddr.sin_port = htons(port_);
    toaddr.sin_addr.s_addr = inet_addr(ip_addr_.c_str());
    
    size_t  len = sendto(g_socketudp, (void *)UCWP_buff, 1206, 0, (struct sockaddr *)&toaddr, sizeof(toaddr));
    close(g_socketudp);
    return true;
}

bool AdvancedSetting::setWakeUp(){
    if(buf_.empty()){return false;}


   if(!checkpacket(buf_))
    {
        std::cout << "error buf_" << std::endl;
        SendFirstPacket();
        return false;
    }

    unsigned char UCWP_buff[1206];

    // 帧头识别符：0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA
    UCWP_buff[0] = 0xAA;
    UCWP_buff[1] = 0x00;
    UCWP_buff[2] = 0xFF;
    UCWP_buff[3] = 0x11;
    UCWP_buff[4] = 0x22;
    UCWP_buff[5] = 0x22;
    UCWP_buff[6] = 0xAA;
    UCWP_buff[7] = 0xAA;

    // 拷贝buf_内容（加锁保护）
    {
        std::lock_guard<std::mutex> lock(buf_mutex_);
        for (int i = 8; i < 46; ++i) {
            UCWP_buff[i] = buf_[i];
        }

        UCWP_buff[46] = 0x00;
        UCWP_buff[47] = 0x00;

        for (int i = 48; i < 1204; ++i) {
            UCWP_buff[i] = buf_[i];
        }
    }
    // 帧尾
    UCWP_buff[1204] = 0x0F;
    UCWP_buff[1205] = 0xF0;

    std::string ip_addr_ = std::to_string(UCWP_buff[10]) + "." +
                           std::to_string(UCWP_buff[11]) + "." + 
                           std::to_string(UCWP_buff[12]) + "." + 
                           std::to_string(UCWP_buff[13]);

    //  std::cout << ip_addr_ <<"    " <<   port_ <<" wake up!!!!" <<std::endl;


    int g_socketudp = socket(2,2,0);
    sockaddr_in toaddr;
    toaddr.sin_family = AF_INET;
    toaddr.sin_port = htons(port_);
    toaddr.sin_addr.s_addr = inet_addr(ip_addr_.c_str());
    
    size_t  len = sendto(g_socketudp, (void *)UCWP_buff, 1206, 0, (struct sockaddr *)&toaddr, sizeof(toaddr));
    close(g_socketudp);

    return true;

}

bool AdvancedSetting::SendFirstPacket()
{
    unsigned char UCWP_buff[1206];

    // 帧头识别符：0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA
    UCWP_buff[0] = 0xAA;
    UCWP_buff[1] = 0x00;
    UCWP_buff[2] = 0xFF;
    UCWP_buff[3] = 0x11;
    UCWP_buff[4] = 0x22;
    UCWP_buff[5] = 0x22;
    UCWP_buff[6] = 0xAA;
    UCWP_buff[7] = 0xAA;

    // 拷贝buf_内容（加锁保护）
    {
        // std::lock_guard<std::mutex> lock(buf_mutex_);
        for (int i = 8; i < 1204; ++i) {
            UCWP_buff[i] = first_packet_buf_[i];
        }
    }
    // 帧尾
    UCWP_buff[1204] = 0x0F;
    UCWP_buff[1205] = 0xF0;

    std::string ip_addr_ = std::to_string(UCWP_buff[10]) + "." +
                           std::to_string(UCWP_buff[11]) + "." + 
                           std::to_string(UCWP_buff[12]) + "." + 
                           std::to_string(UCWP_buff[13]);

    std::cout << ip_addr_ <<"    " <<   port_ <<std::endl;

    int g_socketudp = socket(2,2,0);
    sockaddr_in toaddr;
    toaddr.sin_family = AF_INET;
    toaddr.sin_port = htons(port_);
    toaddr.sin_addr.s_addr = inet_addr(ip_addr_.c_str());
    
    size_t  len = sendto(g_socketudp, (void *)UCWP_buff, 1206, 0, (struct sockaddr *)&toaddr, sizeof(toaddr));
    close(g_socketudp);

    return true;

}

void AdvancedSetting::getStatus() const{


}



} // namespace driver
} // namespace timoo




