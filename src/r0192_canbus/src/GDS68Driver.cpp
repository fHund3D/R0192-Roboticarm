// GDS68Driver.cpp -> Implementation of the GDS68Driver class, which provides methods to control the GDS68 motor controller over CAN bus.
#include "r0192_canbus/GDS68Driver.hpp"
#include <cmath>
#include <cstring>
#include <algorithm> // Für std::clamp
#include <cstdint>

// rev to rad
float revToRad(float rev, int gear_ratio = 8) {
    return rev * 2.0f * M_PI / gear_ratio;
}

// rad to rev
float radToRev(float rad, int gear_ratio = 8) {
    return rad * gear_ratio / (2.0f * M_PI);
}


GDS68Driver::GDS68Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger) 
    : node_id_(node_id), comm_(comm), logger_(logger) {} 


// ------------------ Canbus Protokoll: CAN Simple Protocol (siehe Doku - 4.1) ------------------

// ------------------ Canbus Write ------------------

// CMD ID: 0x001
bool GDS68Driver::Heartbeat() {
}

// CMD ID: 0x002 
bool GDS68Driver::Estop() {
    RCLCPP_WARN(logger_, "Axis %d: EMERGENCY STOP triggered!", node_id_);
    return comm_->sendFrame(createId(0x002), 0, nullptr); // [cite: 5065]
}

// CMD ID: 0x003 
bool GDS68Driver::Get_Error() {
}

// CMD ID: 0x004
bool GDS68Driver::RxSdo() {
}

// CMD ID: 0x005
bool GDS68Driver::TxSdo() {
}

// CMD ID: 0x006
bool GDS68Driver::Set_Axis_Node_ID() {
}

// CMD ID: 0x007
bool GDS68Driver::Set_Axis_State(uint32_t Axis_Requested_State) {
    uint8_t data[8] = {0};
    // Axis_Requested_State:
    // 0: Undefined
    // 1: Idle
    // 3: Calibration (Motor Calibration + Encoder Calibration)
    // 4: Motor Calibration
    // 7: Encoder Calibration
    // 8: Closed-loop
    if (Axis_Requested_State == 1){
        RCLCPP_INFO(logger_, "Axis %d: Set_Axis_State to idle (no power)", node_id_);
    } else if (Axis_Requested_State == 8) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Axis_State to closed loop", node_id_);
    } else {
        RCLCPP_ERROR(logger_, "Axis %d: Error setting Set_Axis_State!", node_id_);
    }
    std::memcpy(&data[0], &Axis_Requested_State, 4);
    return comm_->sendFrame(createId(0x007), 8, data);
}

// CMD ID: 0x008
bool GDS68Driver::mitControl(float Position, float Speed, float KP_Value, float KD_Value, float Torque) {
    
    // 1. Werte limitieren und gemäß GDS68 Protokoll umrechnen 
    
    // Position (16 bit): Skaliert von -12.5 bis 12.5 rad 
    float pos_clamp = std::clamp(Position, -12.5f, 12.5f);
    uint16_t pos_int = static_cast<uint16_t>((pos_clamp + 12.5f) * 65535.0f / 25.0f);
    
    // Speed (12 bit): Skaliert von -65 bis 65 rad/s 
    float speed_clamp = std::clamp(Speed, -65.0f, 65.0f);
    uint16_t vel_int = static_cast<uint16_t>((speed_clamp + 65.0f) * 4095.0f / 130.0f);
    
    // KP (12 bit): Skaliert von 0 bis 500 
    float kp_clamp = std::clamp(KP_Value, 0.0f, 500.0f);
    uint16_t kp_int = static_cast<uint16_t>(kp_clamp * 4095.0f / 500.0f);
    
    // KD (12 bit): Skaliert von 0 bis 5 
    float kd_clamp = std::clamp(KD_Value, 0.0f, 5.0f);
    uint16_t kd_int = static_cast<uint16_t>(kd_clamp * 4095.0f / 5.0f);
    
    // Torque (12 bit): Skaliert von -50 bis 50 Nm 
    float torque_clamp = std::clamp(Torque, -50.0f, 50.0f);
    uint16_t t_int = static_cast<uint16_t>((torque_clamp + 50.0f) * 4095.0f / 100.0f);

    // 2. Werte auf die 8 Bytes des CAN-Frames aufteilen (Bit-Shifting) 
    uint8_t data[8] = {0};
    
    data[0] = (pos_int >> 8) & 0xFF;                                 // Position High 8 bits
    data[1] = pos_int & 0xFF;                                        // Position Low 8 bits
    
    data[2] = (vel_int >> 4) & 0xFF;                                 // Speed High 8 bits
    data[3] = ((vel_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);      // Speed Low 4 bits + KP High 4 bits
    
    data[4] = kp_int & 0xFF;                                         // KP Low 8 bits
    
    data[5] = (kd_int >> 4) & 0xFF;                                  // KD High 8 bits
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);        // KD Low 4 bits + Torque High 4 bits
    
    data[7] = t_int & 0xFF;                                          // Torque Low 8 bits

    RCLCPP_INFO(logger_, "Axis %d: MIT Control - Position: %f rad, Speed: %f rad/s, KP: %f, KD: %f, Torque: %f Nm", node_id_, Position, Speed, KP_Value, KD_Value, Torque);
    return comm_->sendFrame(createId(0x008), 8, data);
}

// CMD ID: 0x009
bool GDS68Driver::Get_Encoder_Estimates() {
    // Sendet nur den Request (Remote Transmission Request wird hier über CMD 0x09 simuliert)
    RCLCPP_DEBUG(logger_, "Axis %d: Requesting Encoder Estimates", node_id_);
    return comm_->sendFrame(createId(0x009), 0, nullptr);
}

// CMD ID: 0x00A
bool GDS68Driver::Get_Encoder_Count() {
}

// CMD ID: 0x00B
bool GDS68Driver::Set_Controller_Mode(uint32_t Control_Mode, uint32_t Input_Mode) {
    uint8_t data[8];
    
    // Control_Mode:
    // 0: Voltage Control
    // 1: Torque Control
    // 2: Speed Control
    // 3: Position control
    if (Control_Mode == 3) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode to Position Control", node_id_);
    }
    else {
        RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode Error - Unsupported mode value: %u", node_id_, Control_Mode);
    }
    std::memcpy(&data[0], &Control_Mode, 4);

    // Input_Mode:
    // 0: Idle
    // 1: Direct Control
    // 2: Speed ramp
    // 3: Position Filtering
    // 5: Trapezoidal curve
    // 6: Torque Ramp
    // 9: Motion Control (MIT)
    if (Input_Mode == 0) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode to Idle", node_id_);
    }
    else if(Input_Mode == 3) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode to Position Filtering", node_id_);
    }
    else if(Input_Mode == 9) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode to Motion Control (MIT)", node_id_);
    }
    else {
        RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode Error - Unsupported mode value: %u", node_id_, Input_Mode);
    }
    std::memcpy(&data[4], &Input_Mode, 4);

    return comm_->sendFrame(createId(0x00B), 8, data);
}

// CMD ID: 0x00C
bool GDS68Driver::Set_Input_Pos(float Input_Pos, uint32_t Duration_ms, float Vel_FF, float Torque_FF) {
    uint8_t data[8];
    float pos_rev = radToRev(Input_Pos, 8);
    int16_t v_ff = static_cast<int16_t>(radToRev(Vel_FF, 8) * 1000.0f);
    int16_t t_ff = static_cast<int16_t>(radToRev(Torque_FF, 8) * 1000.0f);
    
    std::memcpy(&data[0], &pos_rev, 4);
    std::memcpy(&data[4], &v_ff, 2);
    std::memcpy(&data[6], &t_ff, 2);
    RCLCPP_INFO(logger_, "Axis %d: Set_Position - Position: %f rad, Duration: %u ms, Velocity_ff: %f rad/s, Torque_ff: %f Nm", node_id_, Input_Pos, Duration_ms, Vel_FF, Torque_FF);        return comm_->sendFrame(createId(0x00C), 8, data);
}

// CMD ID: 0x00D
bool GDS68Driver::Set_Input_Vel() {    
}

// CMD ID: 0x00E
bool GDS68Driver::Set_Input_Torque() {    
}

// CMD ID: 0x00F
bool GDS68Driver::Set_Limits(float Velocity_Limit, float Current_Limit) {
    uint8_t data[8];
    float vel_limit_rev = radToRev(Velocity_Limit, 8);

    std::memcpy(&data[0], &vel_limit_rev, 4);
    std::memcpy(&data[4], &Current_Limit, 4);
    RCLCPP_INFO(logger_, "Axis %d: Set_Limit - Velocity: %f rad/s, Current: %f A", node_id_, Velocity_Limit, Current_Limit);
    return comm_->sendFrame(createId(0x00F), 8, data);
}

// CMD ID: 0x010
bool GDS68Driver::Start_Anticogging() {    
}

// CMD ID: 0x011
bool GDS68Driver::Set_Traj_Vel_Limit() {    
}

// CMD ID: 0x012
bool GDS68Driver::Set_Traj_Accel_Limits() {    
}

// CMD ID: 0x013
bool GDS68Driver::Set_Traj_Inertia() {    
}

// CMD ID: 0x014
bool GDS68Driver::Get_Iq() {    
}

// CMD ID: 0x015
bool GDS68Driver::Reboot() {    
}

// CMD ID: 0x016
bool GDS68Driver::Set_Input_Torque() {    
}

// CMD ID: 0x017
bool GDS68Driver::Get_Bus_Voltage_Current() {    
}

// CMD ID: 0x018
bool GDS68Driver::Clear_Errors() {
    RCLCPP_WARN(logger_, "Axis %d: Clearing errors!", node_id_);
    return comm_->sendFrame(createId(0x018), 0, nullptr); // [cite: 5180]
}

// CMD ID: 0x019
bool GDS68Driver::Set_Linear_Count() {    
}

// CMD ID: 0x01A
bool GDS68Driver::Set_Pos_Gain() {    
}

// CMD ID: 0x01B
bool GDS68Driver::Set_Vel_Gains() {    
}

// CMD ID: 0x01C
bool GDS68Driver::Get_Torques() {
    return comm_->sendFrame(createId(0x01C), 0, nullptr);
}

// CMD ID: 0x01D
bool GDS68Driver::Get_Powers() {
    return comm_->sendFrame(createId(0x01D), 0, nullptr);
}

// CMD ID: 0x01E
bool GDS68Driver::Disable_Can() {    
}

// CMD ID: 0x01F
bool GDS68Driver::Save_Configuration() {
    RCLCPP_INFO(logger_, "Axis %d: Saving configuration", node_id_);
    return comm_->sendFrame(createId(0x01F), 0, nullptr);
}


// ------------------ Canbus Read ------------------

// Diese Funktion wird aufgerufen, wenn ein Frame mit der passenden ID empfangen wird
void GDS68Driver::processFeedbackFrame(const struct can_frame &frame) {
    // MIT Antwort
    if(frame.can_id == createId(0x008)){  
        // Position: 16 Bit (Byte 1 und 2)
        uint16_t pos_int = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
        
        // Geschwindigkeit: 12 Bit (Byte 3 komplett + obere 4 Bit von Byte 4)
        uint16_t vel_int = (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
        
        // Drehmoment: 12 Bit (untere 4 Bit von Byte 4 + Byte 5 komplett)
        uint16_t t_int   = ((static_cast<uint16_t>(frame.data[4]) & 0x0F) << 8) | frame.data[5];

        // Umrechnung der Integer-Werte in reale Float-Werte 
        // (Exakte Umkehrung der Sender-Logik basierend auf den GDS68 Limits)
        last_pos_rad_  = (static_cast<float>(pos_int) * 25.0f / 65535.0f) - 12.5f;
        last_vel_rads_ = (static_cast<float>(vel_int) * 130.0f / 4095.0f) - 65.0f;
        last_torque_   = (static_cast<float>(t_int) * 100.0f / 4095.0f) - 50.0f;

        RCLCPP_DEBUG(logger_, "Axis %d Feedback: Pos %.2f rad, Vel %.2f rad/s, Torque %.2f Nm", node_id_, last_pos_rad_, last_vel_rads_, last_torque_);

    } else if (frame.can_id == createId(0x009)) {
        // Prüfen, ob die ID zu diesem Motor und dem CMD 0x09 gehört --> Dann die Daten extrahieren und in last_pos_rad_ und last_vel_rads_ speichern
        float pos_rev, vel_unit;
        
        // Bytes zurück in Float kopieren
        std::memcpy(&pos_rev, &frame.data[0], 4);
        std::memcpy(&vel_unit, &frame.data[4], 4);

        // Umrechnen in Radianten (Pos ist in rev, Vel in rad/s)
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_pos_rad_ = revToRad(pos_rev, 8);
        last_vel_rads_ = revToRad(vel_unit, 8);

        RCLCPP_DEBUG(logger_, "Axis %d Feedback: Pos %.2f rad, Vel %.2f rad/s", node_id_, last_pos_rad_, last_vel_rads_);

    } else if (frame.can_id == createId(0x01C)) {
        // Prüfen, ob die ID zu diesem Motor und dem CMD 0x01C gehört --> Dann die Daten extrahieren
        float torque_setpoint, torque;
        
        // Bytes zurück in Float kopieren
        std::memcpy(&torque_setpoint, &frame.data[0], 4);
        std::memcpy(&torque, &frame.data[4], 4);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            //torque_setpoint_ = torque_setpoint;
            last_torque_ = torque;
        }

        RCLCPP_DEBUG(logger_, "Axis %d Feedback: torque %.2f Nm", node_id_, last_torque_);
        
    } else if (frame.can_id == createId(0x01D)) {
        // Prüfen, ob die ID zu diesem Motor und dem CMD 0x01D gehört --> Dann die Daten extrahieren
        float electrical_power, mechanical_power;
        
        // Bytes zurück in Float kopieren
        std::memcpy(&electrical_power, &frame.data[0], 4);
        std::memcpy(&mechanical_power, &frame.data[4], 4);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            electrical_power_ = electrical_power;
            //mechanical_power_ = mechanical_power;
        }

        RCLCPP_DEBUG(logger_, "Axis %d Feedback: Electrical Power %.2f W", node_id_, electrical_power_);
    }
}
