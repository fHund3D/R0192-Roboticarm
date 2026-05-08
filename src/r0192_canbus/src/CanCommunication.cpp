#include "r0192_canbus/CanCommunication.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>

CanCommunication::CanCommunication(const std::string& interface_name) 
    : socket_fd_(-1), interface_name_(interface_name) {}

CanCommunication::~CanCommunication() { if (socket_fd_ >= 0) close(socket_fd_); }

// Funktion zum Initialisieren der Socket-Verbindung
bool CanCommunication::init() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) return false;

    // Socket non-blocking machen
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags == -1) return false;
    if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1) return false;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    return bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) >= 0;
}

// Funktion zum Senden eines Frames
// in CanCommunication.cpp
bool CanCommunication::sendFrame(uint32_t can_id, uint8_t dlc, const uint8_t* data) {
    struct can_frame frame;
    
    // Auto-Detect für Extended Frames (29-Bit)
    if (can_id > 0x7FF) {
        frame.can_id = can_id | CAN_EFF_FLAG; // Setzt das Extended Flag für SocketCAN
    } else {
        frame.can_id = can_id; // Standard 11-Bit Frame
    }
    
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    
    std::lock_guard<std::mutex> lock(send_mutex_);
    return write(socket_fd_, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame);
}

// Funktion zum Lesen eines Frames
bool CanCommunication::readFrame(struct can_frame& frame) {
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100ms Timeout

    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);

    int ret = select(socket_fd_ + 1, &readfds, NULL, NULL, &timeout);
    if (ret > 0 && FD_ISSET(socket_fd_, &readfds)) {
        return read(socket_fd_, &frame, sizeof(struct can_frame)) > 0;
    }
    return false;  // Timeout oder Fehler
}