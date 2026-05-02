// GDS68Driver.hpp -> A driver for the GDS68 motor controller, which communicates over CAN bus.
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "r0192_canbus/CanCommunication.hpp"
#include <memory>
#include <cstdint>
#include <mutex>

class GDS68Driver {
public:
    GDS68Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger);

    bool estop();
    bool setAxisState(uint32_t Axis_Requested_State); // 1=Idle, 8=Closed-Loop
    bool mitControl(float Position, float Speed, float KP_Value, float KD_Value, float Torque);
    bool get_Encoder_Estimates();
    bool setControllerMode(uint32_t Control_Mode, uint32_t Input_Mode); // 1.0 -> Position Control & Position Filtering == TRUE
    bool setPosition(float Input_Pos, uint32_t Duration_ms, float Vel_FF, float Torque_FF);
    bool setLimits(float Velocity_Limit, float Current_Limit);
    bool clearErrors();
    bool get_Torques();
    bool get_Powers();
    bool saveConfiguration();

    // Verarbeitet einen Frame, wenn die ID passt
    void processFeedbackFrame(const struct can_frame &frame);

    // Getter für die Node
    float getLatestPos() { std::lock_guard<std::mutex> lock(data_mutex_); return last_pos_rad_; }   //Position in Radianten
    float getLatestVel() { std::lock_guard<std::mutex> lock(data_mutex_); return last_vel_rads_; }  //Velocity in rad/s
    float getLatestTorque() { std::lock_guard<std::mutex> lock(data_mutex_); return last_torque_; } //Torque in Nm
    // float getLatestTorqueSetpoint() { std::lock_guard<std::mutex> lock(data_mutex_); return torque_setpoint_; } //Torque in Nm
    float getLatestElectricalPower() { std::lock_guard<std::mutex> lock(data_mutex_); return electrical_power_; } //Electrical Power in Watt
    // float getLatestMechanicalPower() { std::lock_guard<std::mutex> lock(data_mutex_); return mechanical_power_; } //Mechanical Power in Watt

private:
    uint8_t node_id_;
    std::shared_ptr<CanCommunication> comm_;
    rclcpp::Logger logger_;

    // Speicher für Feedback
    float last_pos_rad_ = 0.0f;
    float last_vel_rads_ = 0.0f;

    //speicher für Torque 
    //float torque_setpoint_ = 0.0f;
    float last_torque_ = 0.0f;

    // Speicher für Power
    float electrical_power_ = 0.0f;
    // float mechanical_power_ = 0.0f;

    std::mutex data_mutex_; // Schützt vor gleichzeitigem Zugriff (Lesen/Schreiben)

    uint32_t createId(uint8_t cmd_id) { return (node_id_ << 5) + cmd_id; }
};
