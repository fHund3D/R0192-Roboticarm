#include "r0192_canbus/RS05Driver.hpp"
#include <cstring>
#include <cmath>
#include <chrono>


RS05Driver::RS05Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger) 
    : node_id_(node_id), comm_(comm), logger_(logger) {} 

// ------------------ Canbus Protokoll: CAN 2.0 (siehe Doku) ------------------

// ------------------ Canbus Write ------------------

// 4.1.1. Communication type 0: Get device ID
// force_extended=true because comm_type=0x00 makes the ID equal to just node_id_
// (e.g. 4), which is < 0x7FF and would otherwise be sent as a standard 11-bit frame.
bool RS05Driver::Get_Device_ID(){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x00 << 24) | node_id_;
    RCLCPP_DEBUG(logger_, "Axis %d: Get device ID", node_id_);
    return comm_->sendFrame(ext_id, 8, data, true);
}

// 4.1.2. Communication Type 1: operation control mode motor control instruction
bool RS05Driver::MIT_Control(float target_pos, float target_vel, float kp, float kd, float torque){
    
    // 1. Werte limitieren und gemäß Abschnitt 4.1.2 in 16-Bit (0-65535) umrechnen
    
    // Position (16 bit): Skaliert von -4π bis 4π (-12.56637f bis 12.56637f)
    float pos_clamp = std::clamp(target_pos, -12.56637f, 12.56637f);
    uint16_t pos_int = static_cast<uint16_t>((pos_clamp + 12.56637f) * 65535.0f / (2.0f * 12.56637f));
    
    // Speed (16 bit): Skaliert von -50 bis 50 rad/s 
    float speed_clamp = std::clamp(target_vel, -50.0f, 50.0f);
    uint16_t vel_int = static_cast<uint16_t>((speed_clamp + 50.0f) * 65535.0f / 100.0f);
    
    // KP (16 bit): Skaliert von 0 bis 500 
    float kp_clamp = std::clamp(kp, 0.0f, 500.0f);
    uint16_t kp_int = static_cast<uint16_t>(kp_clamp * 65535.0f / 500.0f);
    
    // KD (16 bit): Skaliert von 0 bis 5 
    float kd_clamp = std::clamp(kd, 0.0f, 5.0f);
    uint16_t kd_int = static_cast<uint16_t>(kd_clamp * 65535.0f / 5.0f);
    
    // Torque (16 bit): Skaliert von -5.5 bis 5.5 Nm 
    float torque_clamp = std::clamp(torque, -5.5f, 5.5f);
    uint16_t t_int = static_cast<uint16_t>((torque_clamp + 5.5f) * 65535.0f / 11.0f);

    // 2. Werte auf die 8 Bytes des CAN-Frames aufteilen (Big-Endian laut Handbuch)
    uint8_t data[8] = {0};
    
    data[0] = (pos_int >> 8) & 0xFF;  // Position High 8 bits
    data[1] = pos_int & 0xFF;         // Position Low 8 bits
    
    data[2] = (vel_int >> 8) & 0xFF;  // Speed High 8 bits
    data[3] = vel_int & 0xFF;         // Speed Low 8 bits
    
    data[4] = (kp_int >> 8) & 0xFF;   // KP High 8 bits
    data[5] = kp_int & 0xFF;          // KP Low 8 bits
    
    data[6] = (kd_int >> 8) & 0xFF;   // KD High 8 bits
    data[7] = kd_int & 0xFF;          // KD Low 8 bits

    // 3. Extended ID zusammenbauen (29-Bit)
    // Bit 28~24: Communication Type (0x01)
    // Bit 23~8: Target Torque
    // Bit 7~0: Motor CAN ID
    uint32_t ext_id = (0x01 << 24) | (t_int << 8) | node_id_;

    RCLCPP_DEBUG(logger_, "Axis %d: Op Control - Pos: %.2f, Vel: %.2f, KP: %.2f, KD: %.2f, Torque: %.2f", node_id_, target_pos, target_vel, kp, kd, torque);
    
    // Sende den Frame. Stelle sicher, dass comm_->sendFrame Extended-IDs verarbeiten kann (z.B. CAN_EFF_FLAG bei SocketCAN).
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.4. Communication Type 3: Motor enabled to run
bool RS05Driver::Motor_Enabled_To_Run(){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x03 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Servo on", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.5. Communication Type 4: Motor stops running
bool RS05Driver::Motor_Stop_Running(){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x04 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Servo off", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}

//4.1.6. Communication type 6: Set motor mechanical zero
bool RS05Driver::Set_Motor_Mechanical_Zero(){
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x06 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Set to zero", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}

// Communication type 7: Set motor CAN_ID
bool RS05Driver::Set_Motor_CAN_ID(uint8_t new_id){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x07 << 24) | (new_id << 16) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Set to new ID: %d", node_id_, new_id);
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.7. Communication type 17: Single parameter read
// Request: comm_type 0x11, target motor in bits 7-0; Byte0~1 = parameter index
// (little-endian), Byte2~7 = 0. The motor replies with comm_type 0x11 (parsed in
// processFeedbackFrame). ID = (0x11<<24)|node_id_ > 0x7FF → sent as extended.
// NOTE: index endianness (little-endian here) is the assumption to verify on
// hardware; if no reply arrives, try big-endian (data[0]=high, data[1]=low).
bool RS05Driver::Single_Parameter_Read(uint16_t index){
    uint8_t data[8] = {0};
    data[0] = index & 0xFF;          // index low byte
    data[1] = (index >> 8) & 0xFF;   // index high byte
    uint32_t ext_id = (0x11 << 24) | node_id_;
    RCLCPP_DEBUG(logger_, "Axis %d: Single parameter read index 0x%04X", node_id_, index);
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.8. Communication type 18: Single parameter write (lost in power failure)
bool RS05Driver::Single_Parameter_Write(){
    // nicht implementiert
    return false;
}

// 4.1.10. Communication type 22: Motor data save frame
bool RS05Driver::Motor_Data_Save_Frame(){
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t ext_id = (0x16 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Saving Motor Data", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.11. Communication type 23: Motor baud rate modification frame (re-power-on effect)
bool RS05Driver::Motor_Baudrate_Modification_Frame(float Baudrate){
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};

    if (Baudrate == 1.0f) {
        data[6] = 0x01;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting Baudrate to 1 Mbps", node_id_);
    } else if (Baudrate == 500.0f) {
        data[6] = 0x02;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting Baudrate to 500 kbps", node_id_);
    } else if (Baudrate == 250.0f) {
        data[6] = 0x03;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting Baudrate to 250 kbps", node_id_);
    } else if (Baudrate == 125.0f) {
        data[6] = 0x04;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting Baudrate to 125 kbps", node_id_);        
    } else {
        RCLCPP_ERROR(logger_, "Axis %d: Unsupported Baudrate value: %f kbps", node_id_, Baudrate);
        return false;
    }

    uint32_t ext_id = (0x17 << 24) | node_id_; 
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.12. Communication type 24: The motor actively reports frames
bool RS05Driver::Actively_Reports_Frame(float Report_Time){
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};

    if (Report_Time == 0.0f) {
        data[6] = 0x00;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Disable active motor reporting (default)", node_id_);
    } else if (Report_Time == 1.0f) {
        data[6] = 0x01;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Active motor reporting to 10 ms", node_id_);
    } else {
        RCLCPP_ERROR(logger_, "Axis %d: Unsupported motor reporting", node_id_);
        return false;
    }

    uint32_t ext_id = (0x18 << 24) | node_id_; 
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.13. Communication type 25: Motor protocol modification frame (re-power-on effect)
bool RS05Driver::Motor_Protocol_Modification_Frame(float Can_Protocol){
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    if (Can_Protocol == 0.0f) {
        data[6] = 0x00;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting CAN Protocol to private protocol (default)", node_id_);
    } else if (Can_Protocol == 1.0f) {
        data[6] = 0x01;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting CAN Protocol to Canopen protocol", node_id_);
    } else if (Can_Protocol == 2.0f) {
        data[6] = 0x02;
        data[7] = 0x00;
        RCLCPP_DEBUG(logger_, "Axis %d: Setting CAN Protocol to MIT protocol", node_id_);
    } else {
        RCLCPP_ERROR(logger_, "Axis %d: Unsupported CAN Protocol value: %f", node_id_, Can_Protocol);
        return false;
    }

    uint32_t ext_id = (0x19 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Saving Motor Data", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}

// 4.1.14. Communication type 26：Version number read frame
bool RS05Driver::Version_Number_Read_Frame(){
    uint8_t data[8] = {0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x04 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Get motor version number", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}

//4.1.15. Read and write a single parameter list
bool RS05Driver::Read_And_Write_Single_Parameter_List(std::string Parameter_Name, float Parameter_Value){
    // nicht implementiert
    (void)Parameter_Name; (void)Parameter_Value;
    return false;
}



// ------------------ Canbus Read ------------------

void RS05Driver::processFeedbackFrame(const struct can_frame &frame) {

    // 1. Prüfen, ob es sich um einen Extended Frame (29-Bit) handelt
    if (!(frame.can_id & CAN_EFF_FLAG)) {
        RCLCPP_WARN(logger_, "Received standard frame, but RS05 private protocol expects extended frames!");
        return; 
    }

    // 2. Extrahieren der echten 29-Bit ID (ohne SocketCAN Flags)
    uint32_t ext_id = frame.can_id & CAN_EFF_MASK;
    
    // 3. Extrahieren der Bestandteile der ID laut Handbuch (Abschnitt 4)
    uint8_t comm_type = (ext_id >> 24) & 0x1F;       // Communication Type (Bit 24-28)
    uint8_t sender_id = (ext_id >> 8) & 0xFF;        // Motor ID des Absenders (Bit 8-15)
    
    // Optional: Filterung, falls der Frame nicht zu diesem Knoten gehört
    // if (sender_id != node_id_) return;

    // 4. Verarbeitung basierend auf dem Communication Type
    switch (comm_type) {
        
        case 0x02: { 
            // 4.1.3. Communication Type 2: Motor Feedback Data
            // ODER 4.1.14. Version Number (Das Handbuch sagt, beides antwortet mit Type 2!)
            
            // Check auf Version Number Frame (Byte 0 = 0x00, Byte 1 = 0xC4 laut 4.1.14)
            if (frame.data[0] == 0x00 && frame.data[1] == 0xC4) {
                RCLCPP_DEBUG(logger_, "Axis %d: Version Number %d.%d", sender_id, frame.data[3], frame.data[4]);
                break;
            }

            // Normales Motor Feedback (Ist-Werte)
            // Extrahiere Motor-Status aus der ID (Bit 16-23)
            uint8_t fault_info = (ext_id >> 16) & 0x3F;  // Bit 16-21: Fehlermeldungen
            uint8_t mode_status = (ext_id >> 22) & 0x03; // Bit 22-23: 0=Reset, 1=Cali, 2=Motor Mode

            // Parsen der 8 Daten-Bytes (Big-Endian)
            uint16_t pos_int   = (frame.data[0] << 8) | frame.data[1];
            uint16_t vel_int   = (frame.data[2] << 8) | frame.data[3];
            uint16_t t_int     = (frame.data[4] << 8) | frame.data[5];
            int16_t  temp_int  = (frame.data[6] << 8) | frame.data[7];

            // Rückrechnung in physikalische Einheiten (Umkehrung der Skalierung)
            float current_pos    = (static_cast<float>(pos_int) * (2.0f * 12.56637f) / 65535.0f) - 12.56637f;
            float current_vel    = (static_cast<float>(vel_int) * 100.0f / 65535.0f) - 50.0f;
            float current_torque = (static_cast<float>(t_int) * 11.0f / 65535.0f) - 5.5f;
            float current_temp   = static_cast<float>(temp_int) / 10.0f;

            RCLCPP_DEBUG(logger_, "Axis %d Feedback | Pos: %.2f rad, Vel: %.2f rad/s, Torque: %.2f Nm, Temp: %.1f C | Mode: %d, Fault: 0x%02X", 
                         sender_id, current_pos, current_vel, current_torque, current_temp, mode_status, fault_info);
            
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                current_pos_    = current_pos;
                current_vel_    = current_vel;
                current_torque_ = current_torque;
                current_temp_   = current_temp;
                mode_status_    = mode_status;
                fault_info_     = fault_info;
            }
            break;
        }

        case 0x00: { 
            // 4.1.1. Communication type 0: Get device ID reply
            RCLCPP_DEBUG(logger_, "Axis %d: Device ID Reply received.", sender_id);
            // Payload (frame.data) enthält 64-bit MCU unique identifier
            break;
        }

        case 0x11: {
            // 4.1.7. Communication type 17: Single parameter read reply.
            // Index little-endian (matches the request); value little-endian float
            // in Byte4~7. mechPos/mechVel feed current_pos_/current_vel_ so the
            // joint keeps tracking while the motor is disabled (Reset mode).
            uint8_t success_flag = (ext_id >> 16) & 0xFF; // Bit 16-23: 0x00 = success
            uint16_t index = frame.data[0] | (frame.data[1] << 8);

            if (success_flag == 0x00) {
                uint32_t raw_val = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (static_cast<uint32_t>(frame.data[7]) << 24);
                float param_value;
                std::memcpy(&param_value, &raw_val, sizeof(float));

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    if (index == PARAM_MECH_POS)      current_pos_ = param_value;
                    else if (index == PARAM_MECH_VEL) current_vel_ = param_value;
                }
                RCLCPP_DEBUG(logger_, "Axis %d: Read Param Index 0x%04X = %f", sender_id, index, param_value);
            } else {
                RCLCPP_WARN(logger_, "Axis %d: Read Param Index 0x%04X FAILED", sender_id, index);
            }
            break;
        }

        case 0x15: { 
            // 4.1.9. Communication type 21: Fault feedback frame reply
            // Auslesen der spezifischen Fehlercodes
            uint32_t fault_value = (static_cast<uint32_t>(frame.data[0]) << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
            uint32_t warning_value = (static_cast<uint32_t>(frame.data[4]) << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
            
            RCLCPP_ERROR(logger_, "Axis %d: FAULT FRAME! Fault Code: 0x%08X, Warning Code: 0x%08X", sender_id, fault_value, warning_value);
            break;
        }

        default: {
            // Fängt Dinge ab wie ACK-Frames (z.B. Type 0x16 für Data Save)
            RCLCPP_DEBUG(logger_, "Axis %d: Received CAN frame with comm_type: 0x%02X", sender_id, comm_type);
            break;
        }
    }
}


bool RS05Driver::probePresent(int timeout_ms) {
    Get_Device_ID();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    struct can_frame frame;
    while (std::chrono::steady_clock::now() < deadline) {
        if (comm_->readFrame(frame)) {
            if (frame.can_id & CAN_EFF_FLAG) {
                uint32_t ext_id = frame.can_id & CAN_EFF_MASK;
                uint8_t sender_id = (ext_id >> 8) & 0xFF;
                if (sender_id == node_id_) {
                    RCLCPP_INFO(logger_, "Axis %d (RS05): present — response received", node_id_);
                    return true;
                }
            }
        }
    }
    RCLCPP_WARN(logger_, "Axis %d (RS05): no response within %d ms — treating as virtual", node_id_, timeout_ms);
    return false;
}