#pragma once
#include <rclcpp/rclcpp.hpp>
#include "r0192_canbus/CanCommunication.hpp"
#include <memory>
#include <cstdint>

// --- To DO: Alle Funktionen implementieren, siehe Protokoll ---

class GDS34Driver {
public:
    GDS34Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger);

    bool setPosition(float Input_Pos, uint32_t Duration_ms, float Vel_FF, float Torque_FF);
    bool setControllerMode(uint32_t Control_Mode, uint32_t Input_Mode); // 1.0 -> Position Control & Position Filtering == TRUE
    bool setAxisState(uint32_t Axis_Requested_State); // 1=Idle, 8=Closed-Loop
    bool setLimits(float Velocity_Limit, float Current_Limit);
    bool estop();
    bool clearErrors();
    bool startMotor();
    bool stopMotor();
    bool acknowledgeFault();
    bool get_Encoder_Estimates(uint32_t Flag);
    bool get_Torques();
    bool get_Powers();
    bool saveConfiguration();

private:
    uint8_t node_id_;
    std::shared_ptr<CanCommunication> comm_;
    rclcpp::Logger logger_;
};