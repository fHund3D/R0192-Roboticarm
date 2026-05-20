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
    // force_extended=true: always send as 29-bit extended frame (needed for RS05
    // comm_type 0x00 whose ID would otherwise be < 0x7FF and sent as standard).
    bool sendFrame(uint32_t can_id, uint8_t dlc, const uint8_t* data, bool force_extended = false);
    bool readFrame(struct can_frame& frame);

private:
    int socket_fd_;
    std::string interface_name_;
    std::mutex send_mutex_;
};