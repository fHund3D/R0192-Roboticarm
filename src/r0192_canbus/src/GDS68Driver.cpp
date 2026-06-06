#include "r0192_canbus/GDS68Driver.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <chrono>

// Helper: motor output revolutions ↔ joint radians (gear ratio 8:1)
static float revToRad(float rev) { return rev * 2.0f * M_PI / 8.0f; }
static float radToRev(float rad) { return rad * 8.0f / (2.0f * M_PI); }

GDS68Driver::GDS68Driver(uint8_t node_id, std::shared_ptr<CanCommunication> comm, rclcpp::Logger logger)
    : node_id_(node_id), comm_(comm), logger_(logger) {}

// ------------------ CAN Simple Protocol (GDS68 Doku) ------------------

// CMD 0x001 – Heartbeat request
bool GDS68Driver::Heartbeat() {
    return comm_->sendFrame(createId(0x001), 0, nullptr);
}

// CMD 0x002 – Emergency stop
bool GDS68Driver::Estop() {
    RCLCPP_WARN(logger_, "Axis %d: EMERGENCY STOP triggered!", node_id_);
    return comm_->sendFrame(createId(0x002), 0, nullptr);
}

// CMD 0x003 – Request error state
bool GDS68Driver::Get_Error() {
    return comm_->sendFrame(createId(0x003), 0, nullptr);
}

// CMD 0x007 – Set axis state (1=Idle, 8=Closed-loop)
bool GDS68Driver::Set_Axis_State(uint32_t Axis_Requested_State) {
    uint8_t data[8] = {0};
    if (Axis_Requested_State == 1) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Axis_State → Idle", node_id_);
    } else if (Axis_Requested_State == 8) {
        RCLCPP_INFO(logger_, "Axis %d: Set_Axis_State → Closed-loop", node_id_);
    } else {
        RCLCPP_WARN(logger_, "Axis %d: Set_Axis_State → unknown state %u", node_id_, Axis_Requested_State);
    }
    std::memcpy(&data[0], &Axis_Requested_State, 4);
    return comm_->sendFrame(createId(0x007), 8, data);
}

// CMD 0x008 – MIT impedance control
bool GDS68Driver::MIT_Control(float Position, float Speed, float KP_Value, float KD_Value, float Torque) {
    // Command in homed joint coordinates → shift back into raw encoder frame.
    { std::lock_guard<std::mutex> lock(data_mutex_); Position += home_offset_; }

    // 16-bit position, 12-bit vel/kp/kd/torque per GDS68 protocol
    uint16_t pos_int = static_cast<uint16_t>((std::clamp(Position, -12.5f, 12.5f) + 12.5f) * 65535.0f / 25.0f);
    uint16_t vel_int = static_cast<uint16_t>((std::clamp(Speed,    -65.0f,  65.0f) + 65.0f) * 4095.0f / 130.0f);
    uint16_t kp_int  = static_cast<uint16_t>( std::clamp(KP_Value,   0.0f, 500.0f)          * 4095.0f / 500.0f);
    uint16_t kd_int  = static_cast<uint16_t>( std::clamp(KD_Value,   0.0f,   5.0f)          * 4095.0f / 5.0f);
    uint16_t t_int   = static_cast<uint16_t>((std::clamp(Torque,   -50.0f,  50.0f) + 50.0f) * 4095.0f / 100.0f);

    uint8_t data[8] = {0};
    data[0] = (pos_int >> 8) & 0xFF;
    data[1] =  pos_int       & 0xFF;
    data[2] = (vel_int >> 4) & 0xFF;
    data[3] = ((vel_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
    data[4] =  kp_int        & 0xFF;
    data[5] = (kd_int >> 4)  & 0xFF;
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);
    data[7] =  t_int         & 0xFF;

    RCLCPP_DEBUG(logger_, "Axis %d MIT: pos=%.2f vel=%.2f kp=%.1f kd=%.2f tau=%.2f",
                 node_id_, Position, Speed, KP_Value, KD_Value, Torque);
    return comm_->sendFrame(createId(0x008), 8, data);
}

// CMD 0x009 – Request encoder position + velocity
bool GDS68Driver::Get_Encoder_Estimates() {
    RCLCPP_DEBUG(logger_, "Axis %d: Requesting Encoder Estimates", node_id_);
    return comm_->sendFrame(createId(0x009), 0, nullptr);
}

// CMD 0x00B – Set controller mode
bool GDS68Driver::Set_Controller_Mode(uint32_t Control_Mode, uint32_t Input_Mode) {
    uint8_t data[8];
    std::memcpy(&data[0], &Control_Mode, 4);
    std::memcpy(&data[4], &Input_Mode,   4);
    RCLCPP_INFO(logger_, "Axis %d: Set_Controller_Mode control=%u input=%u", node_id_, Control_Mode, Input_Mode);
    return comm_->sendFrame(createId(0x00B), 8, data);
}

// CMD 0x00C – Set target position with feedforward
bool GDS68Driver::Set_Input_Pos(float Input_Pos, uint32_t /*Duration_ms*/, float Vel_FF, float Torque_FF) {
    uint8_t data[8];
    float pos_rev = radToRev(Input_Pos);
    int16_t v_ff  = static_cast<int16_t>(radToRev(Vel_FF)    * 1000.0f);
    int16_t t_ff  = static_cast<int16_t>(radToRev(Torque_FF) * 1000.0f);
    std::memcpy(&data[0], &pos_rev, 4);
    std::memcpy(&data[4], &v_ff,    2);
    std::memcpy(&data[6], &t_ff,    2);
    RCLCPP_DEBUG(logger_, "Axis %d: Set_Input_Pos pos=%.2f rad", node_id_, Input_Pos);
    return comm_->sendFrame(createId(0x00C), 8, data);
}

// CMD 0x00F – Set velocity and current limits
bool GDS68Driver::Set_Limits(float Velocity_Limit, float Current_Limit) {
    uint8_t data[8];
    float vel_rev = radToRev(Velocity_Limit);
    std::memcpy(&data[0], &vel_rev,        4);
    std::memcpy(&data[4], &Current_Limit,  4);
    RCLCPP_INFO(logger_, "Axis %d: Set_Limits vel=%.2f rad/s current=%.2f A", node_id_, Velocity_Limit, Current_Limit);
    return comm_->sendFrame(createId(0x00F), 8, data);
}

// CMD 0x018 – Clear errors
bool GDS68Driver::Clear_Errors() {
    RCLCPP_WARN(logger_, "Axis %d: Clearing errors", node_id_);
    return comm_->sendFrame(createId(0x018), 0, nullptr);
}

// CMD 0x019 – Zero encoder at current physical position
// Equivalent to odrv0.axis0.encoder.set_linear_count(n) in odrivetool.
// After this call, get_current_position() returns ≈0 and MIT_Control(0.0)
// holds the motor at its current physical location.
bool GDS68Driver::Set_Linear_Count(int32_t linear_count) {
    uint8_t data[4];
    std::memcpy(&data[0], &linear_count, 4);
    RCLCPP_INFO(logger_, "Axis %d: Set_Linear_Count → %d (encoder zeroed at current position)", node_id_, linear_count);
    return comm_->sendFrame(createId(0x019), 4, data);
}

// Set the raw encoder position (rad) that should read as joint 0. After this,
// get_current_position() returns (raw - offset) and MIT_Control() targets are
// shifted by +offset. Works for the absolute encoder where Set_Linear_Count
// has no lasting effect.
void GDS68Driver::set_home_offset(float offset_rad) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    home_offset_ = offset_rad;
    RCLCPP_INFO(logger_, "Axis %d: home_offset set to %.4f rad (raw) → that position now reads 0",
                node_id_, offset_rad);
}

// CMD 0x01C – Request torque feedback
bool GDS68Driver::Get_Torques() {
    return comm_->sendFrame(createId(0x01C), 0, nullptr);
}

// CMD 0x01D – Request power feedback
bool GDS68Driver::Get_Powers() {
    return comm_->sendFrame(createId(0x01D), 0, nullptr);
}

// CMD 0x01F – Save configuration to flash
bool GDS68Driver::Save_Configuration() {
    RCLCPP_INFO(logger_, "Axis %d: Saving configuration", node_id_);
    return comm_->sendFrame(createId(0x01F), 0, nullptr);
}

// ------------------ CAN Read / Feedback ------------------

void GDS68Driver::processFeedbackFrame(const struct can_frame &frame) {

    if (frame.can_id == createId(0x001)) {
        // Heartbeat: axis error (4 bytes) + axis state (1 byte)
        uint32_t axis_error;
        uint8_t  axis_state;
        std::memcpy(&axis_error, &frame.data[0], 4);
        std::memcpy(&axis_state, &frame.data[4], 1);
        std::lock_guard<std::mutex> lock(data_mutex_);
        fault_info_  = static_cast<uint8_t>(axis_error);
        mode_status_ = axis_state;

    } else if (frame.can_id == createId(0x008)) {
        // MIT Control feedback (only arrives while we actively send MIT_Control).
        // Position and velocity are deliberately NOT taken from here anymore:
        // they come exclusively from the periodic Encoder_Estimates frame (0x009,
        // below) so feedback keeps flowing even when the motor is idle/disabled
        // and no MIT_Control is being sent. The MIT field (±12.5 rad) is also a
        // DIFFERENT scale from 0x009 (rev × 2π / gear) — letting both write
        // current_pos_ would make the two frames fight. We only use 0x008 for
        // best-effort torque (0x009 carries no torque).
        uint16_t t_int = ((static_cast<uint16_t>(frame.data[4]) & 0x0F) << 8) | frame.data[5];
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_torque_ = (static_cast<float>(t_int) * 100.0f / 4095.0f) - 50.0f;

    } else if (frame.can_id == createId(0x009)) {
        // Encoder estimates (periodic @ encoder_rate_ms, default 10 ms = 100 Hz):
        // position (rev), velocity (rev/s) as float32. This is now the SOLE
        // source of position/velocity feedback for the GDS68 — available
        // continuously, regardless of control mode or motor enable state.
        float pos_rev, vel_unit;
        std::memcpy(&pos_rev,   &frame.data[0], 4);
        std::memcpy(&vel_unit,  &frame.data[4], 4);
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_pos_ = revToRad(pos_rev);
        current_vel_ = revToRad(vel_unit);
        RCLCPP_DEBUG(logger_, "Axis %d Enc FB: pos=%.2f vel=%.2f", node_id_, current_pos_, current_vel_);

    } else if (frame.can_id == createId(0x01C)) {
        // Torque feedback
        float torque;
        std::memcpy(&torque, &frame.data[4], 4);
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_torque_ = torque;

    } else if (frame.can_id == createId(0x01D)) {
        // Power feedback
        float electrical_power;
        std::memcpy(&electrical_power, &frame.data[0], 4);
        std::lock_guard<std::mutex> lock(data_mutex_);
        electrical_power_ = electrical_power;
    }
}

bool GDS68Driver::probePresent(int timeout_ms) {
    Get_Encoder_Estimates();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    struct can_frame frame;
    while (std::chrono::steady_clock::now() < deadline) {
        if (comm_->readFrame(frame)) {
            if (!(frame.can_id & CAN_EFF_FLAG)) {
                uint16_t std_id = frame.can_id & CAN_SFF_MASK;
                if ((std_id >> 5) == node_id_) {
                    RCLCPP_INFO(logger_, "Axis %d (GDS68): present — response received", node_id_);
                    return true;
                }
            }
        }
    }
    RCLCPP_WARN(logger_, "Axis %d (GDS68): no response within %d ms — treating as virtual", node_id_, timeout_ms);
    return false;
}
