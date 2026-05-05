#include "r0192_canbus/RS05Driver.hpp"
#include <cstring>
#include <cmath>


RS05Driver::RS05Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger) 
    : node_id_(node_id), comm_(comm), logger_(logger) {} 

// Canbus Protokoll: CAN 2.0 --> siehe Doku

bool RS05Driver::Get_Device_ID(){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x00 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Get device ID", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}


bool RS05Driver::MIT_Control(float target_pos, float target_vel, float kp, float kd, float torque){
// Unbedingt ergänzen!!
}


bool RS05Driver::Motor_Enabled_To_Run(){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x03 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Servo on", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}


bool RS05Driver::Motor_Stop_Running(){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x04 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Servo off", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}


bool RS05Driver::Set_Motor_Mechanical_Zero(){
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x06 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Set to zero", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}


bool RS05Driver::Set_Motor_CAN_ID(uint8_t new_id){
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x07 << 24) | (new_id << 16) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Set to new ID: %d", node_id_, new_id);
    return comm_->sendFrame(ext_id, 8, data);
}


bool RS05Driver::Single_Parameter_Read(){
// wird erstmal nicht implementiert 
}


bool RS05Driver::Single_Parameter_Write(){
// wird erstmal nicht implementiert 
}


bool RS05Driver::Fault_Feedback_Frame(){
// muss zu read canbus 
}


bool RS05Driver::Motor_Data_Save_Frame(){
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t ext_id = (0x16 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Saving Motor Data", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}


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


bool RS05Driver::Version_Number_Read_Frame(){
    uint8_t data[8] = {0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t ext_id = (0x04 << 24) | node_id_; 
    RCLCPP_DEBUG(logger_, "Axis %d: Get motor version number", node_id_);
    return comm_->sendFrame(ext_id, 8, data);
}


bool RS05Driver::Read_And_Write_Single_Parameter_List(){
// wird erstmal nicht implementiert
}


// Canbus Read 
void RS05Driver::processFeedbackFrame(const struct can_frame &frame) {
    if(frame.can_id == createId(0x008)){ 

    }
}
