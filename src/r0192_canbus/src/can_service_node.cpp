// can_service_node.cpp -> This node provides a ROS2 service to control the motor controllers via CAN bus, and publishes the state of the first axis at regular intervals.
#include <rclcpp/rclcpp.hpp>
#include <r0192_canbus/srv/motor_command.hpp>
#include <std_msgs/msg/float32.hpp>
#include <map>
#include <memory>
#include <thread>

// Deine neuen Klassen einbinden
#include "r0192_canbus/CanCommunication.hpp"
#include "r0192_canbus/GDS68Driver.hpp"
#include "r0192_canbus/GDS34Driver.hpp"

#include "r0192_canbus/msg/axis_state.hpp"

class CanServiceNode : public rclcpp::Node
{
public:
    CanServiceNode() : Node("can_service_node"), keep_running_(true)
    {
        can_comm_ = std::make_shared<CanCommunication>("can0");
        if (!can_comm_->init()) {
            RCLCPP_ERROR(this->get_logger(), "CAN Socket unable to initialize!");
        }

        drivers_68_[1] = std::make_unique<GDS68Driver>(1, can_comm_, this->get_logger());
        drivers_68_[2] = std::make_unique<GDS68Driver>(2, can_comm_, this->get_logger());
        drivers_68_[3] = std::make_unique<GDS68Driver>(3, can_comm_, this->get_logger());
        drivers_34_[4] = std::make_unique<GDS34Driver>(4, can_comm_, this->get_logger());
        drivers_34_[5] = std::make_unique<GDS34Driver>(5, can_comm_, this->get_logger());
        drivers_34_[6] = std::make_unique<GDS34Driver>(6, can_comm_, this->get_logger());

        service_ = this->create_service<r0192_canbus::srv::MotorCommand>(
            "send_motor_command",
            std::bind(&CanServiceNode::handle_command, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher Map für alle Achsen
        for(int i=1; i<=6; ++i) {
            axis_pubs_[i] = this->create_publisher<r0192_canbus::msg::AxisState>(
                "/axis" + std::to_string(i) + "/state", 10);
        }

        // Timer initialisieren (50Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CanServiceNode::timer_callback, this));

        // Listener Thread starten
        listener_thread_ = std::thread(&CanServiceNode::listen, this);

        RCLCPP_INFO(this->get_logger(), "R0192 Canbus ready to receive commands.");
    }

    ~CanServiceNode() {
        keep_running_ = false;
        if (listener_thread_.joinable()) listener_thread_.join();
    }

private:
    std::shared_ptr<CanCommunication> can_comm_;
    std::map<int, std::unique_ptr<GDS68Driver>> drivers_68_;
    std::map<int, std::unique_ptr<GDS34Driver>> drivers_34_;
    
    // Typen für Map und Thread
    std::map<int, rclcpp::Publisher<r0192_canbus::msg::AxisState>::SharedPtr> axis_pubs_;
    std::thread listener_thread_;
    bool keep_running_;
    
    rclcpp::Service<r0192_canbus::srv::MotorCommand>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Die "Ohr am Gleis" Funktion
    void listen() {
        struct can_frame frame;
        while (keep_running_ && rclcpp::ok()) {
            //RCLCPP_INFO(this->get_logger(), "running");
            if (can_comm_->readFrame(frame)) {
                uint8_t id = frame.can_id >> 5; // Extrahiere Node ID
                if (id >= 1 && id <= 3) drivers_68_[id]->processFeedbackFrame(frame);
                // GDS34 Feedback Logik hier ergänzen --> später!!
            }
        }
    }

    void timer_callback() {
        // Für alle GDS68-Achsen (1-3): Fordere Daten an und publiziere
        for (int id = 1; id <= 3; ++id) {
            auto msg = getAxisStateGDS68(id);
            axis_pubs_[id]->publish(msg);
        }
    }

    // Service-Callback: Verarbeitet eingehende Motorbefehle
    void handle_command(const std::shared_ptr<r0192_canbus::srv::MotorCommand::Request> request, std::shared_ptr<r0192_canbus::srv::MotorCommand::Response> response) {
        int id = request->node_id;
        
        // Sicherheitscheck: Existiert der Treiber?
        if (id < 1 || id > 6) {
            response->success = false;
            response->message = "ID out of range (1-6)";
            return;
        }

        // DISPATCHING: Den richtigen Treiber aufrufen
        if (id <= 3) {
            dispatch_gds68(id, request);
        } else {
            dispatch_gds34(id, request);
        }

        response->success = true;
        response->message = "Command dispatched to driver " + std::to_string(id);
    }

    // Hilfsfunktionen für die dispatch_gds68 für MotorCommand.srv
    void dispatch_gds68(int id, const std::shared_ptr<r0192_canbus::srv::MotorCommand::Request> req) {
        if (req->command_id == 0x002) drivers_68_[id]->estop();
        else if (req->command_id == 0x007) drivers_68_[id]->setAxisState(req->axis_requested_state);
        else if (req->command_id == 0x008) drivers_68_[id]->mitControl(req->value, static_cast<float>(req->velocity), static_cast<float>(req->kp), static_cast<float>(req->kd), static_cast<float>(req->torque));
        else if (req->command_id == 0x009) drivers_68_[id]->get_Encoder_Estimates();
        else if (req->command_id == 0x00B) drivers_68_[id]->setControllerMode(req->control_mode, req->input_mode);
        else if (req->command_id == 0x00C) drivers_68_[id]->setPosition(req->value, req->duration_ms, static_cast<float>(req->velocity), static_cast<float>(req->torque));
        else if (req->command_id == 0x00F) drivers_68_[id]->setLimits(req->velocity, req->torque);
        else if (req->command_id == 0x018) drivers_68_[id]->clearErrors();
        else if (req->command_id == 0x01C) drivers_68_[id]->get_Torques();
        else if (req->command_id == 0x01D) drivers_68_[id]->get_Powers();
        else if (req->command_id == 0x01E) drivers_68_[id]->saveConfiguration();
        else {
            RCLCPP_WARN(this->get_logger(), "Axis %d: Unsupported command ID: 0x%02X", id, req->command_id);
        }
    }

    // Hilfsfunktionen für die dispatch_gds68 für AxisState.msg (ähnlich wie MotorCommand, aber für State-Abruf)
    r0192_canbus::msg::AxisState getAxisStateGDS68(int id) {
        // Driver-Daten anfordern (processFeedbackFrame aktualisiert die Werte im listen() Thread)
        drivers_68_[id]->get_Encoder_Estimates();
        drivers_68_[id]->get_Torques();
        drivers_68_[id]->get_Powers();
        
        // AxisState-Message erstellen und mit Werten aus processFeedbackFrame füllen (über Getter)
        r0192_canbus::msg::AxisState msg;
        msg.node_id = id;
        msg.position = drivers_68_[id]->getLatestPos();                         // Position in Radianten, Rückgabewert von processFeedbackFrame gespeichert
        msg.velocity = drivers_68_[id]->getLatestVel();                         // Velocity in rad/s, Rückgabewert von processFeedbackFrame gespeichert
        msg.torque = drivers_68_[id]->getLatestTorque();                        // Torque in Nm oder A, Rückgabewert von processFeedbackFrame gespeichert
        // msg.torque_setpoint = drivers_68_[id]->getLatestTorqueSetpoint();       // Torque Setpoint in Nm oder A, Rückgabewert von processFeedbackFrame gespeichert
        msg.electrical_power = drivers_68_[id]->getLatestElectricalPower();     // Electrical Power in Watt, Rückgabewert von processFeedbackFrame gespeichert
        // msg.mechanical_power = drivers_68_[id]->getLatestMechanicalPower();     // Mechanical Power in Watt, Rückgabewert von processFeedbackFrame gespeichert
        msg.error_code = 0;  // Placeholder
        msg.axis_state = 0;  // Placeholder
        
        return msg;
    }

    // Hilfsfunktionen für die dispatch_gds34
    void dispatch_gds34(int id, const std::shared_ptr<r0192_canbus::srv::MotorCommand::Request> req) {
        if (req->command_id == 0x95) drivers_34_[id]->setPosition(req->value, req->duration_ms, 0.0f, 0.0f);
        else if (req->command_id == 0x91) drivers_34_[id]->startMotor();
        else if (req->command_id == 0x92) drivers_34_[id]->stopMotor();
        else if (req->command_id == 0x97) drivers_34_[id]->estop();
        else if (req->command_id == 0xB3) drivers_34_[id]->acknowledgeFault();
    }
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanServiceNode>());
    rclcpp::shutdown();
    return 0;
}