#pragma once
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <mutex>

class CanCommunication {
public:
    CanCommunication(const std::string& interface_name);
    ~CanCommunication();

    bool init();
    bool sendFrame(uint32_t can_id, uint8_t dlc, const uint8_t* data);
    bool readFrame(struct can_frame& frame);

private:
    int socket_fd_;
    std::string interface_name_;
    std::mutex send_mutex_;
};