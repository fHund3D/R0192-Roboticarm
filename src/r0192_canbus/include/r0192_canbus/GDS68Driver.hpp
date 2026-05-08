#pragma once
#include <rclcpp/rclcpp.hpp>
#include "r0192_canbus/CanCommunication.hpp"
#include <memory>
#include <cstdint>
#include <mutex>
#include <linux/can.h>

class GDS68Driver {
public:
    GDS68Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger);

    // --- Write Commands ---
    bool Heartbeat(); // CMD 0x001
    bool Estop(); // CMD 0x002
    bool Get_Error(); // CMD 0x003
    bool Set_Axis_State(uint32_t Axis_Requested_State); // CMD 0x007 (1=Idle, 8=Closed-Loop)
    bool MIT_Control(float Position, float Speed, float KP_Value, float KD_Value, float Torque); // CMD 0x008
    bool Get_Encoder_Estimates(); // CMD 0x009
    bool Set_Controller_Mode(uint32_t Control_Mode, uint32_t Input_Mode); // CMD 0x00B
    bool Set_Input_Pos(float Input_Pos, uint32_t Duration_ms, float Vel_FF, float Torque_FF); // CMD 0x00C
    bool Set_Limits(float Velocity_Limit, float Current_Limit); // CMD 0x00F
    bool Clear_Errors(); // CMD 0x018
    bool Get_Torques(); // CMD 0x01C
    bool Get_Powers(); // CMD 0x01D
    bool Save_Configuration(); // CMD 0x01F

    // --- Read / Feedback ---
    void processFeedbackFrame(const struct can_frame &frame);

    // --- Standardisierte Getter für ROS 2 hardware_interface ---
    float get_current_position() { std::lock_guard<std::mutex> lock(data_mutex_); return current_pos_; }
    float get_current_velocity() { std::lock_guard<std::mutex> lock(data_mutex_); return current_vel_; }
    float get_current_torque()   { std::lock_guard<std::mutex> lock(data_mutex_); return current_torque_; }
    float get_current_temp()     { std::lock_guard<std::mutex> lock(data_mutex_); return current_temp_; }
    uint8_t get_mode_status()    { std::lock_guard<std::mutex> lock(data_mutex_); return mode_status_; }
    uint8_t get_fault_info()     { std::lock_guard<std::mutex> lock(data_mutex_); return fault_info_; }
    
    // GDS68 spezifisch
    float get_electrical_power() { std::lock_guard<std::mutex> lock(data_mutex_); return electrical_power_; }

private:
    uint8_t node_id_;
    std::shared_ptr<CanCommunication> comm_;
    rclcpp::Logger logger_;

    std::mutex data_mutex_;

    // --- Variablen zum Speichern der Ist-Werte aus processFeedbackFrame ---
    float current_pos_ = 0.0f;
    float current_vel_ = 0.0f;
    float current_torque_ = 0.0f;
    float current_temp_ = 0.0f;  // Beim GDS68 ggf. über SDO auszulesen, bleibt ansonsten 0.0f
    
    uint8_t mode_status_ = 0;
    uint8_t fault_info_ = 0;
    
    float electrical_power_ = 0.0f;

    // Hilfsfunktion für die ODrive/GDS68 CAN-ID Generierung
    uint32_t createId(uint8_t cmd_id) { return (node_id_ << 5) + cmd_id; }
};
