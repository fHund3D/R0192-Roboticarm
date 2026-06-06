#pragma once

#include <rclcpp/rclcpp.hpp>
#include "r0192_canbus/CanCommunication.hpp"
#include <memory>
#include <cstdint>
#include <string>
#include <mutex>
#include <linux/can.h>

class RS05Driver {
public:
    // Konstruktor
    RS05Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger);

    // --- RS05 Private Protocol (Abschnitt 4) ---

    // 4.1.1. Get device ID
    bool Get_Device_ID();

    // 4.1.2. Operation control mode motor control instruction (Impedanz-/Positionsregelung)
    bool MIT_Control(float target_pos, float target_vel, float kp, float kd, float torque);

    // 4.1.4. Motor enabled to run
    bool Motor_Enabled_To_Run();

    // 4.1.5. Motor stops running (Comm Type 4). With clear_faults=true the stop
    // frame also clears latched faults (Byte[0]=1) — used by the driver-reset
    // path to recover the motor after an emergency stop.
    bool Motor_Stop_Running(bool clear_faults = false);

    // 4.1.6. Set motor mechanical zero
    bool Set_Motor_Mechanical_Zero();

    // Communication type 7: Set motor CAN_ID
    bool Set_Motor_CAN_ID(uint8_t new_id);

    // Readable parameter indices (Type 17 single-parameter read)
    static constexpr uint16_t PARAM_MECH_POS = 0x7019;  // mechanical angle [rad]
    static constexpr uint16_t PARAM_MECH_VEL = 0x701B;  // mechanical velocity [rad/s]

    // 4.1.7. Single parameter read (Type 17). Read-only — does NOT energize the
    // motor, so it works in Reset mode (motor disabled). The async reply is
    // handled in processFeedbackFrame (case 0x11): PARAM_MECH_POS/VEL update
    // current_pos_/current_vel_. Used to keep feedback alive while disabled,
    // since the RS05 has no ODrive-style idle that keeps active reporting running.
    bool Single_Parameter_Read(uint16_t index);

    // 4.1.8. Single parameter write (lost in power failure)
    bool Single_Parameter_Write();

    // 4.1.10. Motor data save frame
    bool Motor_Data_Save_Frame();

    // 4.1.11. Motor baud rate modification frame (re-power-on effect)
    bool Motor_Baudrate_Modification_Frame(float Baudrate);

    // 4.1.12. The motor actively reports frames
    bool Actively_Reports_Frame(float Report_Time);

    // 4.1.13. Motor protocol modification frame (re-power-on effect)
    bool Motor_Protocol_Modification_Frame(float Can_Protocol);

    // 4.1.14. Version number read frame
    bool Version_Number_Read_Frame();

    // 4.1.15. Read and write a single parameter list
    bool Read_And_Write_Single_Parameter_List(std::string Parameter_Name, float Parameter_Value);

    // Returns true if the motor responds within timeout_ms (probe at configure time).
    bool probePresent(int timeout_ms);

    // --- Feedback Processing ---
    
    // Verarbeitet eingehende CAN-Frames vom Motor (Ist-Werte, Fehler, Antworten)
    void processFeedbackFrame(const struct can_frame &frame);

    // --- Getter für ROS 2 hardware_interface ---
    float get_current_position()    { std::lock_guard<std::mutex> lock(data_mutex_); return current_pos_; }
    float get_current_velocity()    { std::lock_guard<std::mutex> lock(data_mutex_); return current_vel_; }
    float get_current_torque()      { std::lock_guard<std::mutex> lock(data_mutex_); return current_torque_; }
    float get_current_temperature() { std::lock_guard<std::mutex> lock(data_mutex_); return current_temp_; }
    uint8_t get_mode_status()       { std::lock_guard<std::mutex> lock(data_mutex_); return mode_status_; }
    uint8_t get_fault_info()        { std::lock_guard<std::mutex> lock(data_mutex_); return fault_info_; }

private:
    uint8_t node_id_;
    std::shared_ptr<CanCommunication> comm_;
    rclcpp::Logger logger_;

    std::mutex data_mutex_;

    // --- Variablen zum Speichern der Ist-Werte aus processFeedbackFrame ---
    float current_pos_ = 0.0f;
    float current_vel_ = 0.0f;
    float current_torque_ = 0.0f;
    float current_temp_ = 0.0f;

    uint8_t mode_status_ = 0;
    uint8_t fault_info_ = 0;
};