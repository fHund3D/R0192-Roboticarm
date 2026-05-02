#include "r0192_canbus/GDS34Driver.hpp"
#include <cstring>
#include <cmath>


GDS34Driver::GDS34Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger) 
    : node_id_(node_id), comm_(comm), logger_(logger) {} 

// CMD ID: 0x92 --> Emergency Stop
bool GDS34Driver::estop(){
    uint8_t data[8];
    data[0] = 0x92;
    std::memset(&data[1], 0, 7);
    RCLCPP_WARN(logger_, "Axis %d: EMERGENCY STOP triggered!", node_id_);
    return comm_->sendFrame(node_id_, 8, data);
}

// CMD ID: 0x01C, Axis_Requested_State = 1 --> Idel, Axis_Requested_State = 8 --> Motor Start/Closed Loop
bool GDS34Driver::setAxisState(uint32_t Axis_Requested_State) {
    uint8_t cmd;
    if (Axis_Requested_State == 1){
        cmd = 0x97;
        RCLCPP_INFO(logger_, "Axis %d: Set_Axis_State to idle (no power)", node_id_);

    } else if (Axis_Requested_State == 8) {
        cmd = 0x91;
        RCLCPP_INFO(logger_, "Axis %d: Set_Axis_State to closed loop", node_id_);
    } else {
        RCLCPP_ERROR(logger_, "Axis %d: Error setting Set_Axis_State!", node_id_);
        return false;
    }

    uint8_t data[8];
    std::memcpy(&data[0], &cmd, 1);
    std::memset(&data[1], 0, 7);
    return comm_->sendFrame(node_id_, 8, data);
}

// CMD ID: 0xB4, Index 0x11 --> Electronical Angle of Rotor (RAD), Index 0x12 --> Mechanical Angle of Rotor (RAD), Index 0x13 --> Mechanical Angle of Output Shaft (RAD)
bool GDS34Driver::get_Encoder_Estimates(uint32_t Flag){
    uint8_t IndID;
    if (Flag == 0) {
        IndID = 0x11; // Electronical Angle of Rotor (RAD)
        RCLCPP_DEBUG(logger_, "Axis %d: Requesting Encoder Estimates (Electronical Angle)", node_id_);

    } else if (Flag == 1) {
        IndID = 0x12; // Mechanical Angle of Rotor (RAD)
        RCLCPP_DEBUG(logger_, "Axis %d: Requesting Encoder Estimates (Mechanical Angle of Rotor)", node_id_);

    } else if (Flag == 2) {
        IndID = 0x13; // Mechanical Angle of Output Shaft (RAD)
        RCLCPP_DEBUG(logger_, "Axis %d: Requesting Encoder Estimates (Mechanical Angle of Output Shaft)", node_id_);

    } else {
        RCLCPP_ERROR(logger_, "Axis %d: Error requesting Encoder Estimates - Unsupported flag value: %u", node_id_, Flag);
        return false;
    }

    uint8_t data[8];
    data[0] = 0xB4;
    std::memcpy(&data[1], &IndID, 1);
    std::memset(&data[2], 0, 6);
    return comm_->sendFrame(node_id_, 8, data);
}

// CMD ID: 0x95, Value: Zielposition in rad, optional Vel_FF und Torque_FF (nicht die eigentlichen vel und toruque, sondern Feedforward Werte - erstmal igniorieren)
bool GDS34Driver::setPosition(float Input_Pos, uint32_t Duration_ms, float Vel_FF, float Torque_FF){

    uint8_t data[8];
    data[0] = 0x95;
    std::memcpy(&data[1], &Input_Pos, 4);
    data[5] = static_cast<uint8_t>(Duration_ms & 0xFF);
    data[6] = static_cast<uint8_t>((Duration_ms >> 8) & 0xFF);
    data[7] = static_cast<uint8_t>((Duration_ms >> 16) & 0xFF);

    RCLCPP_INFO(logger_, "Axis %d: Set_Position - Position: %f rad, Duration: %u ms, Velocity_ff: %f rad/s, Torque_ff: %f Nm", node_id_, Input_Pos, Duration_ms, Vel_FF, Torque_FF);
    return comm_->sendFrame(node_id_, 8, data);
}

// CMD ID: 0xB3, Index 0x02 --> Velocity Limit (RPM), Index 0x01 --> Current Limit (A)
bool GDS34Driver::setLimits(float Velocity_Limit, float Current_Limit){
    // TODO: Implement setLimits
    RCLCPP_WARN(logger_, "Axis %d: setLimits not implemented yet", node_id_);
    return false;
}

// CMD ID 0xB3, Index 0x00 --> Clear Errors
bool GDS34Driver::clearErrors(){
    uint8_t data[8];
    data[0] = 0xB3;
    std::memset(&data[1], 0, 7);
    return comm_->sendFrame(node_id_, 8, data);
}

// CMD ID: 0xB4, Index 0x09 --> Iq (A), Index 0x0A --> Id (A)
bool GDS34Driver::get_Torques(){
    uint8_t data[8];
    data[0] = 0xB4;
    data[1] = 0x09;    // Iq (A)
    std::memset(&data[2], 0, 6);
    return comm_->sendFrame(node_id_, 8, data);
}

// CMD ID: 0xB4, Index 0x15 --> Output Power (W)
bool GDS34Driver::get_Powers(){
    uint8_t data[8];
    data[0] = 0xB4;
    data[1] = 0x15;
    std::memset(&data[2], 0, 6);
    return comm_->sendFrame(node_id_, 8, data);
}

// Gibt es nicht 
bool GDS34Driver::saveConfiguration(){
    // Gibt es nicht 
    return false;
}

bool GDS34Driver::startMotor() {
    return setAxisState(8); // Closed-Loop
}

bool GDS34Driver::stopMotor() {
    return setAxisState(1); // Idle
}

bool GDS34Driver::acknowledgeFault() {
    return clearErrors();
}